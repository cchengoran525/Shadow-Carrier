# Communication — KickPi ↔ C3 通信

> **最终方案：USB 直连。** C3 USB线直连KickPi，CDC ACM 虚拟串口。零额外硬件。

## 架构
```
KickPi USB-A ──USB线──> C3 USB-C
   /dev/ttyACM0           Serial (CDC ACM)
   发 MOVE/STOP/DIFF/PAN/TLT   接收→解析→驱TB6612 + 云台舵机
```

## C3 固件（v0.5）

**规范源码位置：`../C3_USB_Controller/`**（本目录只保留文档，依赖文件不再重复存放）

`C3_USB_Controller.ino` — 在运动控制基础上新增云台：
- `PAN <deg>` / `TLT <deg>`：两轴舵机绝对角度（LEDC 硬件 PWM 50Hz@14bit）
- 超范围自动钳制；回显 `GOT:<cmd>` + `PAN:x.x` 确认
- 方向标定常量 `PAN_INVERT` / `TILT_INVERT`；脉宽标定 `SERVO_PULSE_MIN_US/MAX_US`
- 编译烧录必须带 `CDCOnBoot=cdc`

> 注意：这里的 `Serial` 指 **USB CDC 通道**，不是 C3 的 GPIO10/GPIO11 硬件 UART。
> 根目录 `DistributedRobot_C3_MotionController/` 仍保留原 S3→C3 GPIO UART 固件，
> 两条路径共用 ASCII 命令格式，但不是同一个物理通信入口。

## 协议

| 命令 | 格式 | 说明 |
|------|------|------|
| MOVE | `MOVE F 180\r\n` | 方向 F/B/L/R + 速度 0-255 |
| STOP | `STOP\r\n` | 停车 |
| DIFF | `DIFF L100 R70\r\n` | 差速（跟随模式用） |
| PING | `PING\r\n` | 心跳 |
| PAN | `PAN 90\r\n` | 云台水平角 0~180 |
| TLT | `TLT 45\r\n` | 云台俯仰角 0~180 |

## 踩过的坑
| # | 方案 | 结果 |
|---|------|------|
| 1 | RK板载UART ttyS5/S7/S9 | ❌ TX无物理信号 |
| 2 | CH340 USB-TTL | ❌ 自环通但接C3无信号，OHCI崩溃 |
| 3 | CP2102 USB-TTL | ❌ 同崩溃 |
| 4 | WiFi TCP (C3连热点) | ❌ AP6255 beacon ESP32-C3不可见 |
| 5 | C3 USB直连 | ✅ 即插即用 |

> ⚠️ 教训（2026-08-22）：本目录曾复制了一份依赖文件，与 `../C3_USB_Controller/` 的
> 新版本（带DIFF）产生分叉，一次覆盖导致固件降级、跟随模式失灵。
> **单一真相源原则：依赖只在 `C3_USB_Controller/` 维护。**
