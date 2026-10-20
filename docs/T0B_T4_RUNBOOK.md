# T0b / T4 现场操作单（运动控制，Sprint-1）

> 适用：2026-10-20/21 Sprint-1。机主执行、安全员在场；任何异常先 `STOP`。
> 板端时钟不可信（2023），所有记录时间以**本地 WSL/手机**为准。
> 判据来源：`docs/FIELD_TEST_PLAN.md §3`（T0b / T4）。

---

## 0. 前置检查（2 min，每次开跑都做）

空旷平地，四周 ≥1m，人不要站在转动/直行路径上；地面贴 **0°/90°/180°** 参考线。

板端（SSH 进去执行）：

```bash
bash ~/shadow_carrier_on_rockchip/scripts/health_check.sh   # 必须 FAIL=0
curl -s http://127.0.0.1/ping; echo                          # uart:true, mode:manual
curl -s http://127.0.0.1/mode; echo                          # 必须 manual
curl -s http://127.0.0.1/turn/status; echo                   # active:false, result:idle
```

若 `mode` 不是 manual：

```bash
curl -s http://127.0.0.1/mode/manual; echo
```

在板端先定义两个小函数，后面直接用：

```bash
wait_turn(){ for i in $(seq 1 100); do
  a=$(curl -s http://127.0.0.1/turn/status | python3 -c 'import sys,json;print(json.load(sys.stdin)["active"])')
  [ "$a" = "False" ] && break; sleep 0.1; done
  curl -s http://127.0.0.1/turn/status; echo; }

pivot(){ curl -s -X POST http://127.0.0.1/turn_pivot \
  -H 'Content-Type: application/json' -d "{\"deg\":$1,\"speed\":${2:-190}}"; echo; wait_turn; }
```

---

## 0.5 网页测试台（推荐：拔网线 / 手机操作）

手机连 WiFi **ShadowCarrier-RK**（密码 `shadow123456`）→ 浏览器打开 **`http://192.168.4.1/test`**。
主页面 `/` 底部也有「🧪 测试台」入口；热点下视频流在 `http://192.168.4.1:8080/stream`。

- **① 模式**：切手动（T0b）/ 切跟随（T4）/ 急停 STOP；
- **② T0b**：转 90° / 180° / **-30°（符号确认）** / 自定义 `deg+speed`，实时显示 `/turn/status`；
- **③ T4**：填本条名称 → 开始录制 → 走位 → **停止并分析**（自动出 锁定率/最长缺失/疑似换主）；
- **④ 健康**：fps / age / restarts / `frame_write_fail` / `cam_reopens` / IMU / dnsmasq / 相机节点。

> 以下 §1–§3 的 curl/命令是同一套接口的 CLI 等价写法，二选一即可。

---

## 1. T0b · 原地转标定（10 min）

**目标**：拿到物理 `°/s`、角度误差、重复性；核对 IMU 符号。

**步骤（90° ×3，然后 180° ×3）**：

```bash
pivot 90
pivot 90
pivot 90
pivot 180
pivot 180
pivot 180
```

每次记录（手机秒表 + 视频/量角）：

| # | 指令 | 起始 yaw | 结束 yaw | 耗时 s | 地面实际角度 | 备注 |
|---|---|---|---|---|---|---|
| 1 | 90 | | | | | |
| 2 | 90 | | | | | |
| 3 | 90 | | | | | |
| 4 | 180 | | | | | |
| 5 | 180 | | | | | |
| 6 | 180 | | | | | |

**判定**：
- 角度误差 ≤10°；同向重复性 ±5°；
- 物理角速度 `°/s = 实际角度 / 耗时`，与 `PIVOT_RATE_DPS=174` 偏差 ≤15%；
- `/turn/status` 的 `result` 应为 `ok`（`timeout`/`imu_lost` 记为失败并记录）。

> ⚠️ 说明：当前转向通道用 **IMU 积分闭环**，`PIVOT_RATE_DPS` 在生产代码里**没有消费点**（只在测试/文档）。
> 所以 T0b 的价值是「拿到真实 °/s + 验证 `/turn_pivot`/IMU 符号 + 给 GOTO_SAFE 依据」，**不要为了凑 174 去改常量**；
> 实测值记录进 EXCHANGE 即可。若将来改用时间开环，才需要写回该常量。

**可选（更全）**：`pivot_calib.py` 能一次测 4 档 PWM→°/s、弧线差速、满速 m/s、IMU 符号。
它会**直接开串口发命令**（绕过 rk_control），务必在 manual 且车周无人时跑：

```bash
cd ~/shadow_carrier_on_rockchip/scripts
python3 pivot_calib.py --yes --vmax-dist <米>     # 会原地转 + 直行, 先确认安全
```

---

## 2. GOTO_SAFE 符号确认（2 min）

**目标**：确认「grid bearing 右正 → 车右转（IMU yaw 减小）」，即 `HRI_BEARING_TO_YAW_SIGN=-1` 是否正确。

动作桥里 `target = y0 + (-1) * bearing`；`bearing=+30°（右）`等价于 `/turn_pivot deg=-30`。

```bash
pivot -30        # 车应【向右/顺时针】转 30°
```

- ✅ 车向右 → 符号正确，无需改；
- ❌ 车向左 → 把 `rk_control.py:447` 的 `HRI_BEARING_TO_YAW_SIGN` 改为 `+1.0`，重启 rk-control，并回填 EXCHANGE。

---

## 3. T4 · passby 擦身（20 min，核心）

**前置**：`TURN_ENABLED=True`（10-20 已置）+ `mode=follow` + 认主成功。

```bash
curl -s http://127.0.0.1/mode/follow; echo    # 启动跟随 → 主人站正前 1.5~2m，3s enroll
# 确认已锁定:
cat /dev/shm/owner.json; echo
```

**矩阵**：横向 **0.8 / 1.2 / 1.5 m** × 步速 **0.6（慢）/ 0.9（正常）** × **左/右** = **12 条**。
每条：主人从车**正前 → 侧 → 后**走过；手机全程录像；安全员盯车。

**每条的操作**：

```bash
# 1) 开录(文件名按本条命名, 例 0.8m-慢-左)
python3 ~/shadow_carrier_on_rockchip/scripts/passby_record.py \
  --out ~/hri_logs/passby_0.8_slow_L.jsonl

# 2) 机主走位 + 录像；走完 Ctrl-C 停录

# 3) 自动分析
python3 ~/shadow_carrier_on_rockchip/scripts/passby_analyze.py \
  ~/hri_logs/passby_0.8_slow_L.jsonl
```

**分析脚本自动给**：主人锁定率、最长连续缺失、疑似换主、朝向一致性（粗检）。
**必须手填**（视频 + 卷尺）：停住次数 / 最长停顿（≤1.0s）、横向净空（≥0.6m）、转向是否朝人。

**每条记录表**（§6）：

| 档位 | 停住次数 | 最长停顿 s | 出画时长 s | 净空 m | 换主 | 转向朝人 |
|---|---|---|---|---|---|---|
| 0.8-慢-左 | | | | | | |
| …（共 12 条） | | | | | | |

**判定（全部满足才算过）**：停 ≤1.0s ｜ bbox ≥90% 帧 ｜ 最长缺失 ≤0.5s ｜ 净空 ≥0.6m ｜ **0 换主** ｜ 丢人 ≤2.4s 进 wait_owner。

**任何一条出现「转 30° 就停 / 反向转 / 贴人 <0.6m / 换主」→ 立即停**：

```bash
curl -s http://127.0.0.1/stop; echo
# 立刻抓现场:
bash ~/shadow_carrier_on_rockchip/scripts/health_check.sh
echo kickpi | sudo -S journalctl -u rk-control -n 80 --no-pager | tail -80
```

---

## 4. 收工 / 回退

```bash
curl -s http://127.0.0.1/stop; echo
curl -s http://127.0.0.1/mode/manual; echo
```

- **TURN_ENABLED 回退**：`follow_controller.py:88` 改回 `False`，重启 `rk-control`。
- **相机/链路异常**：`dmesg | grep 0c45`、`~/hri_logs/yolo_daemon.log`、`bash scripts/health_check.sh`。
- 结果按 §9 回执格式写 `EXCHANGE.md`：`[运动控制] ✅/❌ 任务 | 证据 | 板端 md5 | 未过项与下一步`。
