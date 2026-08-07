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

### ⚠️ 编译必加 CDCOnBoot=cdc

ESP32-C3 的 `Serial` 默认走硬件 UART0，必须显式开 USB CDC：

```bash
arduino-cli compile --fqbn "esp32:esp32:esp32c3:CDCOnBoot=cdc" ...
```

不加 → C3 完全沉默，KickPi 写 UART 死锁。

## 开机自启

| 服务 | 端口 | 功能 |
|------|------|------|
| ap-hotspot | WiFi | `ShadowCarrier-RK` / `shadow123456`, NM已排除wlan0 |
| rk-control | 80 | 网页遥控 + USB→C3, by-id路径, udev自动恢复 |
| video-stream | 8080 | YOLO检测 + MJPEG推流 |

## 已知问题

### 1. 冷启动 C3 需插拔 (待定位)
断电重启后 C3 偶尔不被 KickPi 识别，需重新插拔 USB。
可能与电机共享电源有关——C3 和 TB6612 共用电池，电机上电时拉低电压导致 C3 启动失败。
临时方案: 先开 KickPi 等 C3 灯亮，再开电机电源。

### 2. 网站 Safari 重连慢
WiFi 断开重连后，Safari 有时显示"无连接"，需杀掉标签页重开。
根因: MJPEG 流连接卡住浏览器。已改为 JS 延迟加载，仍有偶发问题。

### 3. 视觉跟随: bbox→运动映射 (研究中, 见研究记录)
第一次测试: 20Hz 固定速度开关控制 → 疯狂左右摇摆。
根因: 开关式命令 + 物理系统 ~200ms 响应 + 无滤波 + 无速率限制。
已查到的成熟方案: EMA滤波→比例变速→PD阻尼→速率限制→驻留时间, 五层级联防抖。
详见 `decision/README.md` 跟随控制研究记录。

## 开发路线图

| 阶段 | 状态 |
|------|:--:|
| YOLO管线 (0.8→11fps) | ✅ |
| KickPi热点+HTTP遥控+USB→C3 | ✅ |
| 舵机云台追人 | ✅ |
| 修超声波Bug | ✅ |
| 降延迟 (640x480+Q50+BUFFERSIZE=1) | ✅ |
| 网页模式切换 (遥控/跟随 UI + /move变速接口) | ✅ |
| 视觉跟随闭环 (首次测试→震荡, 待重写控制律) | 🟡 |
| 手动舵机 | 待做 |
| 蓝牙锁主 | 占位 |

## 今日踩坑 (2026-08-07)

### CDCOnBoot=cdc —— 今天最大的坑
ESP32-C3 的 `Serial` 默认走硬件 UART0（C3 Super Mini 上没接）。不加 `CDCOnBoot=cdc` → C3 完全沉默 → KickPi `os.write()` 阻塞 → 双方死锁。**所有后续调试都被这个参数误导了数小时**，每次重编译都漏了它。
教训: arduino-cli 编译 ESP32-C3 时永远带 `--fqbn "esp32:esp32:esp32c3:CDCOnBoot=cdc"`。

### USB CDC 死锁
C3 的 Serial print 输出填满 CDC TX buffer → C3 loop() 阻塞 → C3 不读命令 → KickPi TX buffer 也满 → `os.write()` 阻塞。
修复: C3 固件保留必要打印(诊断用) + rk_control.py 加 uart_reader 后台线程持续排空。

### Python f-string 里的 JS 花括号
JS 的 `{}` 在 Python f-string 里必须写成 `{{}}`。忘记双写 → SyntaxError → 服务挂了。

## 跟人控制研究记录

首次测试(20Hz 开关式命令) → 左右疯狂摇摆。问题拆解:

| 问题 | 方案 |
|------|------|
| bbox 逐帧抖动 → 命令乱跳 | EMA 滤波 (alpha=0.3) |
| 固定速度 → 微偏和大偏一样转 | 比例变速 (偏移大→快转, 偏移小→慢转) |
| 校正过头 → 反复震荡 | PD 阻尼 (D 项检测"正在靠近中心"→减速) |
| 命令切换太快 → 物理跟不上 | 速率限制 + 驻留时间 (≥250ms) |
| 短暂丢人 → 立即停车又起步 | 惯性保持 (丢人后维持上次命令 N 帧) |

控制律: `turn_speed = Kp * offset + Kd * (offset - prev_offset)/dt`  
经 rate limiter 限幅 → `/move {"direction":"L","speed":N}` → C3

下次实现: 五层级联, 先调 EMA 和比例变速, 再加 D 项。
