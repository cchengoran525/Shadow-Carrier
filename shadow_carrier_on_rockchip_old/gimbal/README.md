# Gimbal — 两自由度舵机云台追随

摄像头两轴云台（Pan 水平 / Tilt 垂直），自动追随画面中**面积最大的 person**。
独立于 video_stream_v7 运行，只读它的 HTTP API，**不影响主视频管线**。

## 接线

### 舵机三线

| 舵机线 | 颜色 | 接哪 |
|--------|------|------|
| VCC | 红 | 排针 **5V** |
| GND | 棕/黑 | 排针 **GND** |
| 信号 | 橙/黄 | Pan → **GPIO4_A4**，Tilt → **GPIO4_A5** |

```
┌───────────────────────────────────────────────┐
│ KickPi K11C 30pin 排针                          │
│                                                │
│  5V ──────┬────── 舵机1(VCC红)                  │
│           └────── 舵机2(VCC红)                  │
│  GND ─────┬────── 舵机1(GND棕)                  │
│           └────── 舵机2(GND棕)                  │
│  GPIO4_A6 ────── Pan 舵机1(信号橙)   ← 水平     │
│  GPIO4_A7 ────── Tilt 舵机2(信号橙)  ← 垂直     │
└───────────────────────────────────────────────┘
```

### 引脚调研结论

| 项 | 结果 |
|----|------|
| 硬件 PWM | 仅 pwmchip0 1 路，引脚在 GPIO0_C6（**不在排针**，无法接线） |
| 方案 | **软件 PWM**（GPIO 忙等翻转，50Hz） |
| Pan 信号 | GPIO4_A6（sysfs 134）— pinmux=GPIO，写0/写1有效 ✅ |
| Tilt 信号 | GPIO4_A7（sysfs 135）— pinmux=GPIO，写0/写1有效 ✅ |
| A2/A3 为何不用 | UART7 引脚，**留给未来串口模块** |
| A4/A5 为何不用 | **UART9 引脚，pinmux 被 UART 占用，写 0 无效**（UART TX 空闲拉高） |
| 被占用 GPIO | GPIO4_A0/A1、GPIO3_C6~D7 被 LED 子系统占用，export 失败 |

## 舵机 PWM 控制原理

标准舵机协议（SG90/MG996R 等）：

| 参数 | 值 |
|------|-----|
| 频率 | **50Hz**（周期 20ms） |
| 脉宽 ↔ 角度 | 0.5ms ↔ 0°，1.5ms ↔ 90°，2.5ms ↔ 180° |
| 脉宽公式 | `pulse_ms = 0.5 + angle/180 * 2.0` |

软件 PWM 用 GPIO 忙等实现：拉高脉宽时间，拉低到 20ms 周期。精度 ~0.1ms（≈5°），够"玩玩"。

## 使用

```bash
# 1. 确认 video_stream_v7 在跑 (提供检测API)
curl http://127.0.0.1:8080/api/detections   # 应有 detections

# 2. 导出GPIO并运行 (需要sudo)
sudo python3 gimbal_follow.py
# 或配置NOPASSWD后: python3 gimbal_follow.py

# 3. 看效果: 在摄像头前走动, 云台应跟随你的位置转动
```

## 调参（gimbal_follow.py 顶部常量）

| 常量 | 默认 | 说明 |
|------|------|------|
| `PAN_GPIO` / `TILT_GPIO` | 134 / 135 | 信号引脚 sysfs 号 (GPIO4_A6/A7) |
| `PAN_CENTER` / `TILT_CENTER` | 90 / 90 | 舵机中位角（对准画面中心） |
| `PAN_K` / `TILT_K` | 0.05 / 0.04 | P 控制增益（像素→角度） |
| `SERVO_MIN` / `SERVO_MAX` | 0 / 180 | 角度限幅 |
| `PULSE_MIN` / `PULSE_MAX` | 0.5 / 2.5 | 脉宽范围 ms |

**安装方向不对时**（人走右边舵机却向左）：把对应轴的 `K` 取反（正→负）。

## 控制逻辑

```
每100ms:
  GET /api/detections
  找所有 c=="person" 的框, 取面积最大者
  cx = (x1+x2)/2, cy = (y1+y2)/2     # 人物中心
  dx = cx - 640,  dy = cy - 360      # 相对画面中心偏差(1280x720)
  pan  += dx * PAN_K                 # P控制
  tilt += dy * TILT_K
  限幅到 [SERVO_MIN, SERVO_MAX]

PWM线程持续50Hz输出当前角度, 舵机保持位置
```

## 注意

- **供电**：两个舵机同时动作峰值电流 ~0.5-1A，若板载 5V 不够稳，建议独立 5V 供电，**GND 必须与板子共地**
- **不依赖**：gimbal_follow.py 不碰摄像头、不碰 daemon，只读 HTTP API，v7 可随时停止不影响它
- **占 CPU**：软件 PWM 忙等占 1 个核（4 核板，可接受）
- **无 person 时**：保持当前角度（不追也不回中）
