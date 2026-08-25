# Decision 模块 — 视觉跟随控制设计

## 协议约束

| 命令 | 格式 | C3 行为 |
|------|------|---------|
| 前进 | `MOVE F <0-255>` | Forward |
| 后退 | `MOVE B <0-255>` | Backward |
| 左转 | `MOVE L <0-255>` | **原地旋转**（左轮反+右轮正）|
| 右转 | `MOVE R <0-255>` | **原地旋转** |
| 停止 | `STOP` | 立即停 |

- C3 命令超时 450ms → 大脑必须 ≥2Hz 发命令
- 转向是原地转（坦克转向），不支持弧线
- rk_control.py 提供 `/move {"direction":"F","speed":120}` 变速接口

## 跟人控制: bbox → MOVE 命令

### 首选方案: 五层防抖管线

```
YOLO bbox → [1.EMA滤波] → [2.比例变速] → [3.PD阻尼] → [4.速率限制] → [5.驻留时间] → MOVE
```

### 第 1 层: EMA 平滑

```python
alpha = 0.3  # 平滑系数
smooth_cx = alpha * raw_cx + (1-alpha) * smooth_cx
smooth_h = alpha * raw_h + (1-alpha) * smooth_h
```
砍掉逐帧 3-5px 的 YOLO 抖动。

### 第 2 层: 比例变速

```python
offset = smooth_cx - frame_center
abs_off = abs(offset)

if abs_off <= DEADBAND:  # ~50px
    cmd, spd = "F", 100   # 死区内直走
else:
    ratio = min(abs_off / MAX_OFFSET, 1.0)  # MAX_OFFSET ~200px
    spd = MIN_TURN + ratio * (MAX_TURN - MIN_TURN)  # 80~200
    cmd = "L" if offset < 0 else "R"
```
偏移小 → 慢转，偏移大 → 快转。不再开关式。

### 第 3 层: PD 阻尼

```python
# D 项: 人在向中心靠拢时自动减速，防止转过头
angular = Kp * offset + Kd * (offset - prev_offset) / dt
spd = clamp(abs(angular), MIN_SPD, MAX_SPD)
```
Kp/Kd 需要实测调参。

### 第 4 层: 速率限制

```python
# 每步速度变化不超过 MAX_STEP
spd = clamp(prev_spd - MAX_STEP, target_spd, prev_spd + MAX_STEP)
```
防止瞬时从 0 跳到 200。

### 第 5 层: 驻留时间

```python
# 命令至少保持 DWELL_MS (250ms) 才能切换方向
if now - last_cmd_t < DWELL_MS:
    return  # 不发新命令
```

### 丢失目标: 惯性保持

```python
if person_lost:
    lost_count += 1
    if lost_count < LOST_LIMIT:  # 10-20帧
        cmd = last_cmd   # 惯性保持
    else:
        cmd = "STOP"
```

## 调参表

| 参数 | 建议范围 | 作用 |
|------|---------|------|
| EMA alpha | 0.2-0.4 | 平滑程度 |
| DEADBAND | 40-80px | 中心死区 |
| MIN_TURN | 60-80 | 最小转向 PWM |
| MAX_TURN | 180-220 | 最大转向 PWM |
| MAX_OFFSET | 150-250px | 偏移饱和点 |
| DWELL_MS | 150-300 | 方向最小保持 |
| LOST_LIMIT | 10-20 | 丢人容错帧数 |
| Kp | 0.5-2.0 | 比例增益 |
| Kd | 0.1-0.5 | 阻尼增益 |
| MAX_STEP | 20-40 | 每步最大变速 |

## 外部参考

### Unitree Go2 Follow System (2025)
- **P-only 控制器**（不是 PID），带死区和饱和限制
- 线速度: `vx = 0.9 * min(|error|/1.0, 1.0)`, 死区 1.2m
- 角速度: `wz = 0.96 * min(|error|/1.047rad, 1.0)`, 死区 0.2rad(11.5°)
- 25Hz 循环，无 EMA、无 D 项 — 稳定性来自死区+比例+饱和
- 启示: 即使是商用系统也不需要复杂控制律，核心是**合适的死区和比例映射**

### IBVS (Image-Based Visual Servoing) 简化公式
```
omega = -Kp_angular * horizontal_pixel_error
v     = -Kp_linear  * vertical_pixel_error
```
像素误差直接映射到角/线速度，增益从相机内参和深度估计推导。

### Mini Pupper ROS2 (MangDang)
- YOLO11n ONNX + motpy 多目标追踪
- PID 偏航/俯仰追踪 + IMU 反馈
- Nav2 velocity_smoother: 死区钳位 + 加速度限制 + 插值平滑

### RVPF-YOLO (2025)
- bbox 质心反馈 + 改进 DWA 局部规划器
- 动态权重调整 + 路径跟随子函数
- 规划器与控制器的速度空间统一，消除对抗震荡

## 首次测试教训

20Hz 固定速度开关控制 → 疯狂左右摇摆。原因：
1. 开关式命令 (MOVE L 150 / MOVE R 150) — 无比例
2. 控制频率过高 (20Hz vs 物理响应 ~200ms)
3. 无滤波 (bbox 抖动直接→命令)
4. 无速率限制 (瞬时 0→150)
5. 无驻留时间 (命令切换太快)

五层级联方案覆盖以上所有问题点。

---

# 认主/锁主 (2026-08-24, v0 已部署)

> 设计原则: 每次进入跟随模式都重新快速认主(机主拍板); 认主只换"跟谁", 不动运动控制律; 失败自动降级旧行为。

## 架构

```
点「跟随」→ 阻塞式认主 ~3s:
    连续采样 /api/detections + /dev/shm 原始帧 (~12个样本)
    取每帧最大 person 的 bbox → HSV直方图 + 宽高比
    → OwnerProfile (直方图均值 + 体态比中位数)
之后每个控制周期:
    对画面每个 person 打分 = 0.7×颜色相似度(Bhattacharyya) + 0.3×体态一致度
                            + 连续性加分(≤0.08, 距上帧目标中心<80px线性衰减)
    → 最高分且 ≥SCORE_MIN(0.40) 者 = 主人; 无人过线 → 视为丢失
无模板/模块异常 → 自动降级为旧逻辑"跟画面最大的人"
```

## 文件

| 文件 | 内容 |
|---|---|
| `scripts/owner_id.py` | enroll() / select_target() / read_recent_frame()，调参全在顶部 |
| `scripts/follow_controller.py` v4 | start() 里跑 enroll；_fetch_person() 按模板选人；_info() 加 owner/score 字段 |
| `config/settings.yaml` | owner 段仍是占位，BLE 验证后填真实 MAC |

调参旋钮: `ENROLL_DURATION_S=3.0` `ENROLL_INTERVAL_S=0.25` `H_BINS=30,S_BINS=32` `W_COLOR=0.7` `W_ASPECT=0.3` `SCORE_MIN=0.40` `CONT_BONUS=0.08` `NEAR_PX=80`

## BLE 锁主侦察结论 (2026-08-24)

- **板载 AP6255 蓝牙免 root 可用**（BlueZ D-Bus），原"C3 固件加 BLE 扫描"方案作废，BLE 全在 RK 端做 → [固件] 线的待办可划掉
- 手环9 (`04:34:C3:15:AE:0E`) 连接手机时完全隐身（定向广播+RPA随机地址）；**断连/解绑后走 Mi Beacon 公开广播，带名字+真实 MAC**。已实测验证（2026-08-24 出厂重置后）: ~2.5s/次广播、RSSI -38~-71、ServiceData fe95
- 未连接的 Band 6 实测 ~21s 才广播 1 次 → **BLE 只能做 30~60s 慢速在场心跳（丢主兜底），不能做实时目标锚定**
- ✅ 已实现: `owner_id.start_ble_heartbeat()` 后台每 ~40s 扫一轮记 last_seen；follow_controller 视觉丢主时若手环 60s 内出现过 → `wait_owner` 原地等待，手环也消失才 lost_stop
- ⚠️ demo 前提: 主人手环保持未绑定/未连接状态，否则隐身

## 测试清单

1. 单人: 站车前 1~2m → 点跟随 → 前 3s 认主窗口(车不动) → 正常走动验证跟对人
2. 双人: 第二人入画/遮挡 → 应继续跟原目标（同色系衣服时可能漂移）
3. 日志: `sudo journalctl -u rk-control -n 20` 看 `[owner] enroll samples=N` 与 `[follow] owner lock OK/FAIL`

## 已知边界

- 同色系两人靠体态比+连续性硬撑，可能漂移；逆光/黑白灰衣服直方图信息量下降
- 板上 scripts/__pycache__ 被 root 占用 → 验证语法用 `PYTHONDONTWRITEBYTECODE=1 python3 -c "import owner_id"`
- sudo 密码 = kickpi（默认）

## 认主演进路线 (2026-08-24 调研, 按 demo 性能预算排序)

> 性能前提: YOLO(NPU) 之后要与 OpenCV 快环交替跑, 认主侧不允许增加重计算。
> 已排除: 步态识别(太研究向)、BLE AoA(需天线阵列)、毫米波雷达(无身份信息)。

| 方案 | 内容 | 成本 | 状态 |
|------|------|------|------|
| ④ 简易目标跟踪 | α-β滤波预测 + 门控关联, 替代逐帧独立argmax, 根治相似双人目标漂移 | 纯算法, 几十次浮点运算/帧 | ✅ 已做 |
| ① ReID 嵌入 | OSNet类小模型→512维外观向量替换HSV直方图, 抗光照/更衣 | 模型转RKNN, ~1天 | 备选 |
| ② 人脸确认 | zoo现成RetinaFace(+ArcFace嵌入), 认主瞬间强确认; 跟随多看背影只当注册用 | 低(模型现成) | 锦上添花 |
| ③ 骨架体态 | yolov8_pose关键点算肩髋比/肢长比替代bbox宽高比; 顺手给方向四提供"面向我"信号 | pose模型占NPU时间 ⚠️与快环冲突 | 缓,等性能预算明确 |
| ⑤ UWB测距 | DWM3000一对¥50挂C3(SPI), ±10cm@10Hz穿遮挡, 真实距离喂follow_distance | 硬件+驱动 | 硬件杀手锏,答辩亮点 |

注: BLE 心跳定位不变——只做在场门控, 不参与打分(RSSI噪声±20dB不适合逐帧加权)。

### 调研详记 (2026-08-24)

#### 背景
现代机器人/跟随产品的"认主"技术栈, 按与本项目硬件(RK3566 NPU + C3 + 手环)的契合度评估。

#### 1. 行人重识别 Re-ID —— 学院派标准答案
- 做法: 检测框内人像 → OSNet/TransReID 等网络编码成 512 维嵌入向量 → 余弦相似度比对, 跨摄像头/跨时段认出同一人
- 与现状关系: 我们手搓的 HSV 直方图就是它的手工特征版。升级后抗光照、抗换衣微调、语义比颜色丰富(包/发型都进向量)
- RKNN 可行性: OSNet 是 backbone 级小模型, 社区有 RKNN 转换先例, NPU 推理 <20ms
- 代价: 找权重→转RKNN→改 owner_id 的模板结构, 约 1 天

#### 2. 人脸识别 —— 注册强确认, 不适合持续跟踪
- zoo 里现成 `examples/RetinaFace`(检测), 配 ArcFace 嵌入做 1:1 确认
- 服务机器人( Pepper / 商场导购 )的标准用法: 人脸做"注册+确认", 不做持续跟踪
- 关键短板: **跟随场景大部分时间看的是背影**; 只在主人回头/认主瞬间有用
- 定位: 认主仪式的加分确认项

#### 3. 骨架关键点体态 —— 替掉脆弱的宽高比
- zoo 里现成 `examples/yolov8_pose`, 17关键点 → 肩髋比/四肢长度比例做体型描述子
- bbox 宽高比的已知毛病(README 前文记录): 弯腰/蹲下/侧身就崩 → 骨架比例对姿态稳得多
- 附带收益: 关键点直接给出"主人是否面向我"——方向四 RECEIVE 行为正需要这个信号, 一鱼两吃
- ⚠️ 顾虑: pose 模型占 NPU 时间, 与之后 OpenCV 快环交替跑的性能预算冲突, 缓

#### 4. 正规 MOT 跟踪 —— 已实现(简化版)
- ByteTrack 一族标准流程: 运动模型预测下一帧位置 → 全局最优关联 → 嵌入画廊随时间更新
- 本项目简化为: α-β滤波恒速预测 + 连续性加分 + 220px硬门控(距预测过远的候选重罚0.5分)
- 解决问题: 相似双人时的目标漂移/逐帧独立argmax打架; 重捕获时旧预测自动失效
- 开销: 每帧几十次浮点加减, 忽略不计

#### 5. UWB 测距 —— 跟随行李箱行业的答案 ⭐
- 业界产品(Cowarobot R1 / Airwheel 跟随行李箱)标配: DW1000/DWM3000 模块一对 ¥50, SPI 接主机
- 对比:

| | BLE RSSI(现在) | UWB |
|---|---|---|
| 测距精度 | ±2m 且 ±20dB乱跳 | ±10cm |
| 刷新率 | ~0.05Hz(未连接手环) | 10Hz+ |
| 遮挡影响 | 大 | 小(NLOS可用) |
| 能拿到的信息 | 在场/不在场 | 主人实时距离米数 |

- 直接收益: settings.yaml 里 `follow_distance: 1.0米` 从"靠bbox高度猜"变成真测距; 方向四的 RECEIVE 停车距离也有了硬依据
- 接入点: C3 有空闲 GPIO/SPI, DWM3000 挂 C3, 测距值走现有串口遥测通道上报(将来加 TEL 命令)

#### 排除项及理由
- 步态识别: 研究向, 数据采集和模型都不现实
- BLE AoA 定向: 需要 antenna array 硬件, 复杂度爆炸
- 毫米波雷达(LD2410类): 只有存在/运动信息, 无身份, 不能认主

#### 性能测算 (2026-08-24 估, 上机需bench核实)

前提: RK3566 NPU 0.8TOPS, YOLOv8 单帧52ms → 满速~19fps, 当前13fps → NPU空闲占空比~30%。
之后 YOLO 2Hz + OpenCV 8fps(CPU侧) 时 NPU 空闲 >90%, 空间更大。

关键认知: ReID/Pose 都是 **per-person 裁剪小模型**, 搭 YOLO 检测结果便车,
成本 = 人数 × 调用频率, 与视频帧率无关。

| 模型 | 输入 | 单次推理(估) | 省法 | 实际占用 |
|---|---|---|---|---|
| ReID (OSNet-x0.25) | 128×256裁剪 | ~8–15ms | 歧义触发: 画面≥2人才算(90%时间0调用) | ≤6% 占空比 |
| Pose (RTMPose-t裁剪式) | 192×256裁剪 | ~15–30ms | 低频1Hz + 只算主人轨 | ≤3% 占空比 |

组合最坏情况: YOLO 2Hz + ReID≤6% + Pose≤3% → NPU总占空比 <30%, 有余量。

省法细则:
- ReID: 单人场景无认错可能→零调用; 模板向量认主时算好存着, 逐帧只算候选人
- Pose: 体态比例秒级变化, 1Hz够; 兼职输出"主人面向我"(方向四要用)

注意:
1. 推理耗时为估算值(同类模型RK3566公开benchmark), 落地前先循环100次计时bench
2. 别放CPU跑(OSNet CPU要50–100ms), 必须走NPU
3. ReID 与 HSV 模板接口同构(只换 score_box 一个函数), 优先做; Pose 等快环架构定型再动

## 首轮真人实测记录 (2026-08-24深夜)

### 结果
- 认主采样稳定 (enroll samples=11), BLE心跳正常启动, 三人同框未跟丢
- 舒适度较旧版明显提升 (幻影人过滤后不再追家具)
- **主要丢人原因已明确: 人出摄像头FOV** (跟随模式云台固定回中, 不随动)

### 修复史 (每条都是实测踩出来的)

| # | Bug | 现象 | 修复 |
|---|-----|------|------|
| 1 | `_roi_hist` 把 dict 当 list 迭代 | 画面有人必炸→静默降级"跟最大" | 显式按 key 取值 |
| 2 | `CONT_BONUS` 重构时丢失定义 | 每帧选人抛 NameError, 同样静默降级 | 补常量; 选人异常改为打印可见 |
| 3 | yolo_daemon 偶发坐标序列化成字符串 | 间歇性打分崩溃 | 入口统一强转 float, 坏条目丢弃 |
| 4 | 错锁陷阱: SNAP 门控罚 0.5 分 | 跟错人后真主人永无翻身(距预测>220px即重罚) | 罚分降至 0.15 + 连续~3s低分自动清跟踪器(逃逸重捕获) |
| 5 | 幻影人: YOLO对椅子/包以0.25~0.5误报person | 追凳子背包 / 真人反而不追 | `CONF_MIN=0.50` + `MIN_BOX_H=70px` 双过滤 |
| 6 | bbox 高度做测距两次翻车(阈值拍脑袋/腿部特写框高失真) | 只转不走 / 又撞脚 | **回滚v3恒速**, 测距交给超声波兜底(C3硬件层), 远期UWB |

### 教训
- 静默降级是双刃剑: 保住了可用性, 但掩盖了认主根本没在跑的事实(两轮测试实际都是降级模式)。
  以后关键路径异常必须打到日志并暴露到 /ping。
- 打分制 vs 最大值制: 打分制给了认主能力, 但也把 YOLO 的低置信度垃圾一起放进来,
  必须配置信度地板。

### 当前参数 (scripts/owner_id.py 顶部)
```
SCORE_MIN=0.40  W_COLOR=0.7  W_ASPECT=0.3 (模板饱和度<40自动切换成0.3/0.7)
CONT_BONUS=0.08  NEAR_PX=80px  SNAP_GATE_PX=220(罚0.15)
CONF_MIN=0.50   MIN_BOX_H=70px  ENROLL_DURATION_S=3.0
BAND_MAC=04:34:C3:15:AE:0E  BAND_FRESH_S=60
```

### 下一步 (按对"追得住"的贡献排序)
1. **云台随动进跟随模式** — 人出FOV是当前最大丢因, 云台±90°可覆盖大部分横走场景
   (gimbal_follow.py v2 已备好, 注意与 rk_control 的串口独占问题)
2. 坐标解码(方向二) — 出框前预测运动趋势提前转向
3. wait_owner / 双人同色对抗 实战验证
4. ReID 嵌入替换 HSV (演进路线①)

### 主人携带物演进: 手环 → 手机 (2026-08-25)

| 方案 | 原理 | 状态 |
|------|------|------|
| A. 热点RSSI (零携带) | 遥控时手机必连车热点; WiFi随机MAC按SSID固定→身份稳定。实测: AP6255 AP模式 `iw station dump` 无输出(驱动限制), 改用 **邻居表MAC匹配+ping判活**, 已验证 | ✅ 已部署验证(2026-08-25), iPhone8 MAC=b6:ac:ee:7e:b2:fd |
| B. 自研App后台广播 | App以BLE peripheral广播, 身份=包内自定义UUID而非MAC → 天然绕开RPA随机化; Android后台可行(需电池白名单), iOS限制多 | 📋 已记录, 产品阶段做 |
| C. 手机自带UWB | 2021+旗舰机(iPhone11+/小米12+)内置UWB芯片, 装App授权即可复用DWM3000基站软件 | ⏳ 待硬件: Mi 9无UWB芯片无法试, 需借/换支持机型 |

方案A注意: 覆盖半径≈热点覆盖(~30m); "手机断开热点"成为比手环更干净的"主人离开"信号;
PHONE_MAC 首次使用流程: 手机连热点 → 日志会打印所有关联站的MAC和信号 → 辨认后填入 owner_id.py。
