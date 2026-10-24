# 实地试验应急预案（Field Trial Playbook）

> 写于 2026-10-19，面向 demo 前的每一次走廊/宿舍实机试验。
> 定位：`DEMO_PLAN` §3 风险表的**可执行版**；补上 `DEMO_PLAN` 引用但一直不存在的 `RECOVERY.md`。
> 用法：出事先看 §1（60 秒速查）；每天开工看 §4（检查单）；收工看 §8（事后 30 分钟）。

**图例**：✅ 实机验证过 · ⚠️ 未在实机验证 · 🆕 本轮新增、尚未上板

---

## 0. 现场角色与红线

| 角色 | 谁 | 职责 |
|---|---|---|
| **操作员** | 你 | 发指令、切模式、看日志；**只有操作员能发运动指令** |
| **安全员** | 同学 A | 只盯车与人，手放急停位（`/stop` 或直接断电/抬车）；**有权叫停一切** |
| **被跟随者** | 同学 B | 按动线走；不突然折返、不贴近 <1m 挤过 |
| **拍摄** | 同学 C | 手机固定机位 + 手持跟拍；同时拍**屏幕上的检测框** |

**红线（任何一条被触碰立即停）**
1. **人车距离 <0.5m 且车在动** → 停。
2. **车不听 `/stop`** → 抬车/断 C3 USB，不再尝试软件。
3. **`pivot_calib --yes`、`turn_pivot`、原地转** 时，人不在车体对角线半径内（≥1m）。
4. **不在门槛、台阶、电梯口、湿地面**做自动跟随。
5. 现场改动任何常量后，**没跑离线回归不许再上板**（见 §8.4）。

---

## 1. 60 秒速查（出事了先做这三步）

```bash
# ① 先停（任何时候都有效，不受模式门禁限制）
curl -s -X POST 127.0.0.1:80/stop

# ② 抢回底盘（跟随时手动端点被锁，必须先切模式）
curl -s -X POST 127.0.0.1:80/mode/manual
curl -s 127.0.0.1:80/ping          # 期望 {"uart":true,"mode":"manual"}

# ③ 看一眼是谁坏了
bash ~/shadow_carrier_on_rockchip/scripts/health_check.sh
```

三步之后按 §5 的故障树对号入座。**若 ① 无效 → 直接断电/断 C3 USB（硬件急停）。**

---

## 2. 现场分级（每一级都有明确判据）

| 级 | 状态 | 进入条件 | 切换 |
|---|---|---|---|
| **L0 全自动** | FOLLOW 自动跟随 | `health_check` 全绿 + 认主成功 + 人车距 ≥1.2m | `POST /mode/follow` |
| **L1 限速自动** | 跟随但慢 | 检测 `age_ms` 偶发偏高 / 地面湿 / 人多 | **临时限速需统筹批准**（V1 速度冻结；改完必须跑回归）或由雷达包络限速 |
| **L2 手动** | 人开车 | 行为异常（乱转/跟错人/不跟随）、构图需要 | `POST /mode/manual` + `/forward /back /left /right` |
| **L3 停机** | 车不动、只录视频 | 硬件故障无法排除；demo 当天兜底 | 断电；素材改用已录 |

> **demo 当天的兜底台词**：L2 手动走完剧本 + L3 视频兜底（`DEMO_PLAN` §3 风险 8 原文）。

---

## 3. 每次试验前必须知道的三件事（诚实交底）

| 项 | 现状 | 对现场意味着什么 |
|---|---|---|
| **转向通道** `TURN_ENABLED` | **True（2026-10-20 启用）**；实机 passby 待 T4 验收 ⚠️ | 已能"转头保持视野"；若出现反向转/抖动，先 `/stop`，用 `/motion/config` 降 `PIVOT`，再跑 T4 |
| **停驻转头接人** `tick(turn_only=True)` | **HRI 已是 systemd 服务**（`hri-state`，2026-10-20），`/dev/shm/hri_state.json` 持续更新 ✅ | 递物时车会按 HIDE/WAIT 做有界转头；服务异常看 `systemctl status hri-state`；退路仍是主人站正前递物 |
| **雷达安全过滤** | 惰性契约：无 `/dev/shm/lidar_safety.json` = **逐字节直通** ✅ | 雷达没装/daemon 没跑时，一切行为与旧版完全一致；装了之后要盯 `GET /safety` |

---

## 4. 每日开工检查单（按顺序，10 分钟）

### 4.1 链路（✅ 已验证项）
```bash
bash ~/shadow_carrier_on_rockchip/scripts/health_check.sh
```
必须全 PASS：
- `rk-control / video-stream / ap-hotspot / imu / c3-watchdog / camera-usb-fix` 六个服务 active
- `/dev/camera0` 存在（不是悬空）
- USB 相机 `power/control=on`（`camera-usb-fix` 在保）
- `/api/detections` 有响应且 `age_ms` 低于阈值、`input_age_ms` 不缺失
- `yolo_daemon` 存活、无僵尸、`cam_reopens` 不增长
- `dnsmasq` CPU 正常、`hostapd` 在跑、`wlan0=192.168.4.1`

### 4.2 运动与状态
```bash
curl -s 127.0.0.1:80/ping                 # {"uart":true,"mode":"manual"}
curl -s 127.0.0.1:80/mode
cat /dev/shm/imu.json                     # yaw_deg/yaw_rate_dps 新鲜(age<0.5s)
ls -l /dev/shm/owner.json /dev/shm/hri_state.json 2>/dev/null
```
手动点动一次：`/forward`（IMU 直线保持）→ 车走直线；`/left` `/right` → 原地转；`/stop` 停。
**四向都试一遍再进 L0。**（⚠️ `_manual_forward` 是本轮新增，实机首次要确认方向对）

### 4.3 标定（每次换电池/换载重/首次上板）
```bash
python3 ~/shadow_carrier_on_rockchip/scripts/pivot_calib.py --check-only   # 只看符号, 不动
python3 ~/shadow_carrier_on_rockchip/scripts/pivot_calib.py --yes \
        --vmax-dist <卷尺量到的米数>
```
产出 `calib.json` + 写回清单（`PIVOT_RATE_DPS` / `v_at_spd_max`）。
**只有标定完，才考虑把 `TURN_ENABLED` 置 True。**

### 4.4 认主
面对镜头 3 秒 → `POST /mode/follow` → 看 `journalctl -u rk-control -f` 出现 `[follow] started` 与 `state -> active`。

---

## 5. 故障树（症状 → 判断 → 处理 → 回退）

### A. 链路层

| # | 症状 | 快速判断 | 处理 | 回退 |
|---|---|---|---|---|
| A1 | 网页/API 全无响应 | `curl 127.0.0.1:80/ping` 失败 | `systemctl restart rk-control`；无限重启看 `journalctl -u rk-control -n 100` | 手动断电重启板子 |
| A2 | 检测框不动 / `age_ms` 很大 | health_check 报 `age_ms` 或 `input_age_ms` 缺失 | `systemctl restart video-stream`（连带 yolo）；查 `/dev/camera0` | 若反复：查 USB 线/口，跑一次 `camera-usb-fix` 的检查 |
| A3 | 相机每 ~5 分钟断一次 | `dmesg | grep -i disconnect` 有周期性枚举；`cam_reopens` 增长 | 确认 `camera-usb-fix` active、`power/control=on` | 换 USB 口/线；临时换机位 |
| A4 | 网页活着但电机不动、云台不出声 | `curl /ping` → `uart:false` | `ls -l /dev/ttyACM0`；`c3-watchdog` 日志是否在 usbreset | C3 从 USB 消失（掉电）→ 独立供电/加电容；或断电池重上电 |
| A5 | IMU 数据过期 / `imu_lost` | `cat /dev/shm/imu.json` 的 ts 差 >0.5s | `systemctl restart imu`；`POST /imu/zero`（车必须静止） | 无 IMU 时：follow 的直行/原地转会降级，**改用 L2 手动** |
| A6 | 热点挂、手机连不上 | `hostapd` 未跑 / 无 `192.168.4.1` | `systemctl restart ap-hotspot`（**会断手机，最后才做**） | 用板子本地终端/串口操作 |

### B. 决策层

| # | 症状 | 快速判断 | 处理 | 回退 |
|---|---|---|---|---|
| B1 | follow 已在 follow 模式但车不动 | `journalctl` 里 `[follow] state` 停在 `lost`/`lost_stop`/`wait_owner` | 让人回到正前方 1.5–2.5m；看 `owner.json` 是否新鲜；必要时 `/mode/manual` 后重新认主再 `/mode/follow` | L2 手动 |
| B2 | 启动即 `认主失败x2 → 拒绝进入跟随` | 日志原文 | 穿鲜艳衣物、面对镜头 3 秒、光照够；再 `/mode/follow` | L2 手动 + 不演示认主 |
| B3 | 车速忽快忽慢 / 追不上 | 改过 `BASE_SPD` 或电池电压低 | 查电压；`pivot_calib` 测 vmax；未标定前**不要改 BASE** | 降级 L1/L2 |
| B4 | 递物不触发 RECEIVE | 主人站在车侧方（>32°）→ 相机看不到 | **让主人绕到车正前方 1.2–1.5m 递物**；或手动起 `hri_state.py --live` 让 `hri_state.json` 出现（此时车才可能转头接人） | L2 手动完成交接镜头 |
| B5 | 雷达过滤导致"不走/一直限速" | `curl /safety` 看 `blocked`/`limits`/`env_age_s` | 若判定不合理：`rm -f /dev/shm/lidar_safety.json`（**立即恢复直通**）或停 daemon | 关雷达，回到旧行为 |
| B6 | `POST /turn_pivot` 返回 `error` | `GET /turn/status` | 常见：`follow` 模式（切 manual）/ IMU 不可用（A5）/ busy | 手动 `/left` `/right` 点动 |

### C. 行为层（跟人特有）

| # | 症状 | 快速判断 | 处理 | 回退 |
|---|---|---|---|---|
| C1 | **擦身时停住不动** | 日志 `offset` 与 `smooth_h`；是否 `too_close` | 已知工况：近距 + 偏轴时框高门只在正前 11° 生效。**这是当前设计，不是故障** | 手动带过；或按 §6.1 换走位 |
| C2 | **跟错人（锁到路人）** | 看视频里橙框是否跳到别人；日志 `换人待确认` | **立即 `/stop`** → L2；保留视频与日志 | 重新认主；单人对角站位彩排 |
| C3 | **原地转不停 / 转完还在转** | `GET /turn/status` 的 `yaw/err`；`[follow] 原地转结束` 是否出现 | 预算 180°/超时 6s 应自动收尾；若不停 → `/stop` + manual | L2 |
| C4 | **方向左右摇摆** | 日志 `转向通道进入 TURN dir=L/R` 反复出现 | 反转看门狗（5s>3 次）应强制回弧线；若仍摇摆 → manual | 关闭 `TURN_ENABLED` 回到纯弧线 |
| C5 | **丢人后一直在后退** | 日志 `search_back` | 后退最长 1.2s；若后方是墙 → 立即 manual | L2 |
| C6 | **该停不停（贴近人）** | 目测 <0.7m 仍在走 | 立即 `/stop`；检查 `NEAR_STOP_M`/速度门是否被改 | L2；考虑临时降 BASE |
| C7 | 扫描/搜索朝墙转 | 视频 | 属预期（当前按出画侧扫）；无碰撞风险就让它转完 | manual 中止 |

### D. 安全层

| # | 症状 | 处理（按顺序） |
|---|---|---|
| D1 | 车失控 / 不响应 `/stop` | ① 抬车或断电；② `systemctl stop rk-control`；③ 拔 C3 USB。**不追软件。** |
| D2 | 车贴上人 | `/stop` + 人后退；记录时间点，事后查 `C6` |
| D3 | 原地转扫到人或物 | 超声否决 + 雷达 `rot` 包络应拦；若未拦 → 立即 manual，事后查 `GET /safety` 的 `rot_gap_m` |
| D4 | 手动端点返回 `手动运动端点已锁` | 预期行为（follow 模式独占底盘）：先 `POST /mode/manual`；`/stop` 例外永远可用 |
| D5 | 电池电压低 / BMS 保护 | 立即停；换备用包；不要"再跑一条" |

### E. 数据层

| # | 症状 | 处理 |
|---|---|---|
| E1 | 事后没有日志 | 现场就用 `journalctl -u rk-control --since "10 min ago" > run_$(date +%H%M).log`；同时手机拍屏幕 |
| E2 | 视频没录到检测框 | 每次跑都用手机拍**网页的检测框**（网页每 0.5s 刷新）；这是唯一能回看的"感知真相" |
| E3 | 无法复现 | 记录：时间、位置（第几块瓷砖）、人在哪、车状态、`/mode`、`owner.json` 新鲜度；能复现就录 30s |

---

## 6. 专项预案（按镜头）

### 6.1 passby（主人从旁边走过）
- **现实工况**：走廊横向余量 0.8–1.5m（不是 2m）。
- **当前行为（`TURN_ENABLED=True`，2026-10-20 起）**：人出画侧 → 转向通道 TURN/BLIND/SEARCH 保视野；近距框高门只认正前，侧向不再误停。**待 T4 12 条实测**。
- **旧行为（历史）**：`TURN_ENABLED=False` 时为"弧线跟 → 出画 → 后退 1.2s → 停车等"，镜头会卡住。
- **预案 A（安全优先）**：若 T4 出现反向转/贴人，立即 `/stop`，拍"车停住等人回来"。
- **预案 B**：用 `/motion/config` 降 `PIVOT`/`BASE` 后重跑；`pivot_calib` 仍用于拿真实 °/s。
- **走位**：主人从车**侧后 1.2–1.5m** 绕过；不要贴着 0.5m 挤过（`C6`）。
- **机位**：第三人从车后方侧拍，同时拍到人与车的相对位置。

### 6.2 递物 / RECEIVE
- 主人站在**车正前方 1.2–1.5m**（不要从侧方 76° 来，相机看不到）。
- 手上道具用**袋子/玩偶**（果冻不可检）。
- 若 `RECEIVE` 不出现：先看 `owner.json` 是否新鲜、`dets` 里有没有 person；仍不行 → L2 手动完成交接，画面只拍手部特写（不需要车动）。

### 6.3 HIDE / 选点
- HIDE 依赖 `grid.json` 新鲜（TTL 30s）与 `geometry_daemon`；**实机 HIDE 端到端未验** ⚠️。
- 2026-10-20 起：`hri-state` 常驻 + `geometry-parked.timer` 每 20s 在停驻域刷新 grid；先看 `grid.json` 的 `ts` 是否 <60s。
- 预案：车手动挪到墙根，台词不变（"它自己找了个不挡路的地方"改口播为"它停到墙边"）。

### 6.4 门 / 电梯 / 门槛
- **不做自动跟随**：门槛、台阶、电梯口一律 L2。
- 电梯：人先上，车手动跟上；不要在电梯口让人站在车前。

### 6.5 人多 / 路人交叉
- 先停机等人散；必须拍"锁主不跟路人"时，路人**固定路线、与主人保持 >1.5m**。
- 出现误锁 → C2 流程（`/stop` + L2 + 保留视频）。

### 6.6 逆光 / 夜间 / 反光地面
- **不在夜景做跟随**（历史 95/98 帧含幻影）。
- 逆光：换机位/换时段；玻璃门反光：避开该段动线。

---

## 7. 硬件与环境应急

| 项 | 现象 | 处理 |
|---|---|---|
| **电池** | 电压跌、速度骤降、BMS 保护 | 停；换包；记录电压与运行时长（这也是标定 `v_at_spd_max` 的输入） |
| **电机/舵机过热** | 烫手、异响、扭矩下降 | 停 5–10 分钟；不要在过热状态跑 `pivot_calib` |
| **线材** | USB 周期性断（A3）、串口掉（A4） | 弯折处加扎带；备一根 USB 线；C3 线不要拽 |
| **地面** | 地毯阻力大、瓷砖缝卡轮、湿滑 | 降低预期速度；地毯段改 L2；湿地面不跑 |
| **天气** | 雨/湿 | 只在室内；室外试验取消 |
| **人身** | 围观/儿童/宠物 | 安全员清场；车与人保持 ≥1m |

---

## 8. 事后 30 分钟（每次都做）

### 8.1 收证据
```bash
journalctl -u rk-control --since "30 min ago" > ~/runs/$(date +%Y%m%d_%H%M)_rk.log
journalctl -u video-stream --since "30 min ago" > ~/runs/$(date +%Y%m%d_%H%M)_video.log
journalctl -u imu --since "30 min ago" > ~/runs/$(date +%Y%m%d_%H%M)_imu.log
cp /dev/shm/owner.json /dev/shm/imu.json /dev/shm/hri_state.json ~/runs/ 2>/dev/null
```
外加：手机视频（含网页检测框录屏）、手写记录（时间/位置/现象）。

### 8.2 当天复现一次
能复现的故障，30 秒内重放一次并录下来——**"能复现"比"修好了"更值钱**。

### 8.3 归档与登记
- 结论写进 `EXCHANGE.md`（本地，不入 git）：日期 / 现象 / 根因 / 改动 / 是否复现。
- 若改了代码或常量 → 见 8.4。

### 8.4 改过东西之后的强制回归
```bash
cd ~/shadow_carrier_on_rockchip
for t in scripts/test_turn_pivot.py scripts/test_follow_dt.py scripts/test_weak_cue.py \
         scripts/test_passby_lock_confirm.py scripts/test_hri_receive_close.py \
         scripts/test_control_glue.py world/fusion/test_lidar_guard.py \
         world/fusion/test_bearing_convention.py state/test_tracker_fake.py \
         world/fusion/test_sector_score.py; do
  python3 "$t" >/dev/null 2>&1 && echo "PASS $t" || echo "FAIL $t"
done
```
**全绿才允许下一次上板。**

---

## 9. 命令速查

| 目的 | 命令 |
|---|---|
| 急停 | `curl -X POST 127.0.0.1:80/stop` |
| 切手动 / 跟随 | `POST /mode/manual` / `POST /mode/follow` |
| 手动前进（IMU 直线保持）/ 后退 / 原地转 | `POST /forward` `/back` `/left` `/right` |
| 原地转到角度（仅手动） | `POST /turn_pivot -d '{"deg":180}'`；`GET /turn/status` |
| 弧线转身（仅手动，旧版） | `POST /turn -d '{"deg":90}'` |
| IMU 重零（车静止） | `POST /imu/zero` |
| 雷达安全状态 | `GET /safety` |
| 关闭雷达过滤（直通） | `rm -f /dev/shm/lidar_safety.json` |
| 链路体检 | `bash scripts/health_check.sh` |
| 标定（静止核对 / 全量） | `pivot_calib.py --check-only` / `--yes` |

---

## 10. 一页 Checklist（可打印）

```
[ ] health_check 全绿（六服务 + 相机 + 检测 + 热点）
[ ] /ping 返回 uart:true, mode:manual
[ ] 手动四向点动 OK，/stop 有效
[ ] imu.json 新鲜；/imu/zero 已做（车静止时）
[ ] 认主 3s 成功（/mode/follow 后日志 started）
[ ] 雷达：/safety 符合预期（或确认直通）
[ ] 人车距与走位讲清楚（≥1.2m，不从侧方 0.5m 挤过）
[ ] 安全员就位，手放急停
[ ] 手机在录（含网页检测框）
[ ] 收工：journalctl 已存、视频已存、EXCHANGE 已记
```

---

## 附：本预案与其它文档的关系

| 文档 | 关系 |
|---|---|
| `DEMO_PLAN.md` §3 | 风险表（本文件是它的可执行版） |
| `V2_LIDAR_AVOIDANCE.md` §6 | 雷达失效清单（本文件 §5-B5 / D3 引用它） |
| `DEFERRED_WORK.md` §1.4 | 短时遮挡 vs 持续失联的分层（本文件 §5-C 的行为依据） |
| `ARCHITECTURE.md` §4 | 冻结接口（`/safety`、`owner.json`、`hri_state.json` 的定义） |
| `shadow_carrier_on_rockchip/README.md` | 各模块行为与默认关开关 |
