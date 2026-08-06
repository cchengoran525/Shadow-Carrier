# Shadow Carrier on Rockchip

KickPi (RK3566) 小车大脑。USB直连C3，YOLO NPU + HTTP遥控 + 舵机云台。

## 架构
```
shadow_carrier_on_rockchip/
├── perception/       # YOLO NPU检测 (11fps, watchdog)
│   ├── yolo_daemon.cc / yolo_daemon
│   └── camera.cpp / yolov8.cpp
├── scripts/
│   ├── rk_control.py      # HTTP遥控 (80端口, USB→C3)
│   ├── video_stream_v7.py # MJPEG推流 (8080端口)
│   └── video_stream.py / video_stream_fast.py (旧版)
├── decision/         # 视觉跟随+蓝牙锁主 设计文档
├── communication/    # C3 USB固件 + 协议 + 踩坑记录
├── gimbal/           # 两轴舵机云台 (追最大person)
├── config/           # settings.yaml
└── models/           # (空)
```

## 通信：USB直连 (最终方案)
C3 USB-C → KickPi USB-A，CDC ACM。即供电又通信。无额外硬件。

## 开发路线图
| 阶段 | 状态 |
|------|:--:|
| YOLO管线 (0.8→11fps) | ✅ |
| KickPi热点+遥控+USB→C3 | ✅ |
| 舵机云台追人 | ✅ |
| 视觉跟随闭环 | 待做 |
| 蓝牙锁主 | 占位 |

## 开机自启
| 服务 | 端口 | 功能 |
|------|------|------|
| ap-hotspot | WiFi | ShadowCarrier-RK / shadow123456 |
| rk-control | 80 | 网页遥控+USB→C3 |
| video-stream | 8080 | YOLO+MJPEG推流 |
