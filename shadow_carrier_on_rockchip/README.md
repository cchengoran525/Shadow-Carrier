# Shadow Carrier on Rockchip

KickPi (RK3566) 小车大脑。USB 直连 C3，YOLO NPU 检测(11fps) + HTTP 遥控 + 舵机云台。

## 架构

shadow_carrier_on_rockchip/
├── perception/       # YOLO NPU 检测管线 (~11fps, watchdog)
│   ├── yolo_daemon.cc / yolo_daemon  # C++ 常驻daemon
│   └── camera.cpp / yolov8.cpp       # TODO
├── scripts/
│   ├── rk_control.py          # HTTP 遥控面板 (80, USB→C3)
│   ├── video_stream_v7.py     # MJPEG 推流+检测 (8080)
│   └── video_stream.py / video_stream_fast.py (旧版参考)
├── communication/    # C3 USB 固件 + 协议 + 踩坑记录
│   ├── C3_USB_Controller.ino  # C3 USB 接收固件
│   └── README.md
├── decision/         # 视觉跟随 + 蓝牙锁主 设计文档
├── gimbal/           # 两轴舵机云台 (追最大person)
│   ├── gimbal_follow.py
│   └── README.md
├── config/           # settings.yaml
└── models/           # (空, 模型路径引用)

## 通信: USB 直连 (最终方案)

C3 USB-C → KickPi USB-A，CDC ACM 虚拟串口 `/dev/ttyACM0`。即供电又通信。
by-id 永久路径、udev 插拔自动重启。无需 CH340/WiFi/板载UART。
踩坑顺序: RK UART无TX → CH340 OHCI崩溃 → CP2102崩溃 → AP6255 ESP32不兼容 → USB直连✅

## 开机自启

| 服务 | 端口 | 功能 |
|------|------|------|
| ap-hotspot | WiFi | `ShadowCarrier-RK` / `shadow123456`, NM已排除wlan0 |
| rk-control | 80 | 网页遥控 + USB→C3, by-id路径, udev自动恢复 |
| video-stream | 8080 | YOLO检测 + MJPEG推流 |

## 已知问题 & 下一步

### 1. ⚠️ 超声波传感器 Bug (高优先级)
撞墙后 C3 锁死不再响应，需重新插拔 C3 USB 恢复。
现象: 超声波触发 20cm 挡 Forward → 之后所有命令失效。
待做: 修复 C3_USB_Controller.ino 超声波处理逻辑。

### 2. ⚠️ 延迟 ~1s (高优先级)
端到端延迟接近 1s，远超 YOLO 推理时间(~60ms)。
根因: 视频管线缓冲叠加 (cam buf→daemon→out.jpg→stream)。
待做: 降分辨率/降质量/减少管线缓冲层数。

### 3. 网页模式切换
遥控页增加"手动遥控 / 自动跟随"两种模式切换。
手动 = 按钮发 MOVE；自动 = 视觉闭环发 MOVE。

### 4. 视觉跟随闭环
YOLO person → 状态机 → MOVE 指令 → C3。
decision/README.md 已有完整设计。

### 5. 手动舵机控制
遥控模式下用方向按钮控制云台舵机 (pan/tilt)。
gimbal/ 已有独立程序，需集成到 rk_control.py。

## 开发路线图

| 阶段 | 状态 |
|------|:--:|
| YOLO管线 (0.8→11fps) | ✅ |
| KickPi热点+HTTP遥控+USB→C3 | ✅ |
| 舵机云台追人 | ✅ |
| 修超声波Bug | 🔴 下一步 |
| 降延迟 | 🔴 下一步 |
| 网页模式切换 | 待做 |
| 视觉跟随闭环 | 待做 |
| 手动舵机 | 待做 |
| 蓝牙锁主 | 占位 |
