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
