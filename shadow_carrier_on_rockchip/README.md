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
| 视觉跟随闭环 (差速弧线跟人, 基本可用) | ✅ |
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

## 跟人控制 v3: 差速弧线 (2026-08-08)

### 为什么坦克转向不行

C3 的 MOVE L/R 是原地转（左轮后+右轮前）。即使最慢速度 70，200ms 脉冲也会转过
一大截 → 人从左边跑右边 → 触发反向转 → 来回摇摆 → 丢人。

三轮迭代:
- v1: 固定速度开关控制 → 疯狂摇摆
- v2: 定时脉冲 + 方向滞回 → 一步一停, 跌跌撞撞
- v3: **差速弧线** → 不停车, 边前进边微调方向 ✅

### 差速方案

新增 C3 协议命令 `DIFF L<左速> R<右速>` — 左右轮独立速度, 同向前进 = 弧线。

```
偏移 60px:  DIFF L80 R100 (缓左弧)
偏移 120px: DIFF L50 R100 (中左弧)
偏移 200px: DIFF L30 R100 (急左弧)
```

### C3 固件改动

| 文件 | 改动 |
|------|------|
| Protocol.h | +Diff 命令类型, +leftSpeed/rightSpeed |
| CommandParser.cpp | +DIFF Lxxx Rxxx 解析 |
| MotorDriver.h/cpp | +differential(), +Diff 模式独立斜坡 |

### 已知小问题
- 1:48 电机低速有死区 (~PWM50 以下不动), 微调时内侧轮可能停转
- 弯腰/蹲下时 bbox 高度异常可能触发误判
- 无编码器闭环, 轮速不一致时可能走偏

## 下一步: MVP → Demo 的四个方向

Demo 场景: 跟到饮水机 → 躲门边 → 跟到冰箱 → 接果冻 → 跟回去

```
┌─────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐
│ 方向一   │    │ 方向三   │    │ 方向四   │    │ 方向二   │
│ 锁主     │    │ 世界理解  │    │ 物体接收  │    │ 云台坐标  │
│ 🔴 demo  │    │ 🔴 demo  │    │ 🟡 demo  │    │ 🟢 远期   │
│ 关键      │    │ 关键      │    │ 必要      │    │ 锦上添花  │
└─────────┘    └─────────┘    └─────────┘    └─────────┘
```

### 方向一: 特征识别锁主 (BLE + HSV + 姿势)

**目标**: 跟主人, 不跟路人。

**方案**: 三道轻量信号融合 (无需训练新模型)

| 信号 | 来源 | 作用 |
|------|------|------|
| BLE RSSI | 手机广播 → C3 扫描 | "谁离手机最近" → 主人身份 |
| HSV 颜色直方图 | OpenCV (YOLO bbox 内取色) | "谁穿着和刚才一样的衣服" |
| 姿势/体型 | bbox 宽高比 | "谁的身材和刚才一致" |

- BLE 给身份 (主人手机 MAC 唯一), HSV 给外观连续性, 姿势给一致性
- 单个信号不可靠, 三者融合就够了
- Demo 场景: 主人是唯一带手机的人, 区分路人绰绰有余
- **无需任何模型训练**, 纯 OpenCV + C3 BLE 扫描

**第一步**: C3 固件加 BLE 扫描, KickPi 收 RSSI 值, 先验证"画面里谁离手机最近"这个基本逻辑。

### 方向三: 单目世界理解 (YOLO + OpenCV 串通)

**目标**: 知道周围有什么——门在哪、墙在哪、前面能不能走。

**方案**: YOLO 慢环(语义) + OpenCV 快环(几何) → 局部网格

```
YOLO (NPU, ~3Hz): person在哪, 场景类型, 物体
      ↓
OpenCV (CPU, 30fps): 门框线/地面纹理/边缘异常/消失点
      ↓
局部极坐标网格: 8扇区 × 3距离环, 每格={自由/人/门/墙/未知}
      ↓
行为引擎: FOLLOW / HIDE / STOP
```

| 任务 | 方法 | 难度 |
|------|------|:--:|
| 门检测 | Canny → Hough → 垂直线配对 → 矩形筛选 | 低 |
| 墙-地交界 | 画面下部 Canny → 最长水平线 | 低 |
| 地面可通行 | 边缘密度 + 纹理方差 | 中 |
| 消失点估计 | HoughLines 交点 = 走廊方向 | 中 |

**第一步**: 拍一张有门的照片, OpenCV Canny+Hough 离线验证门检测。

### 方向四: 物体感知 + RECEIVE 行为

**目标**: 主人拿果冻 → 车主动靠近接收。

**方案**: YOLO 已支持 bottle/cup/backpack 等类。融合信号:

| 信号 | 含义 |
|------|------|
| 主人面向机器人 | 脸/b眼分布 = 正面朝向 |
| 主人手中有物 | YOLO 检测到 bottle/cup 等 |
| 主人静止/靠近 | bbox 大小不变或变大 |

三信号同时满足 → RECEIVE: 低速前进 → 超声波 20cm → STOP → 等放物品 → 确认

**第一步**: 验证 YOLO 对常见手持物品(果冻/水瓶)的检测率和置信度。

### 方向二: 云台 + 坐标解糅 (远期)

**目标**: 用云台角度 + bbox 推算人在机器人坐标系中的精确位置。

**方案**: 相机标定(焦距/主点/畸变) → 手眼标定(云台轴/机器人坐标系) → IBVS 公式映射。需要标定板、精确测量, 且廉价舵机精度有限 (±5°)。

**判断**: demo 不需要厘米级定位。差速弧线已在固定摄像头下稳定跟人。后续需要精确空间感知(如 HIDE 精准贴墙)时再做。

### 方向五: 路径记忆 (远期占位)

**目标**: 记得来时的路。跟回去时有导航冗余, 人丢了也能沿路返回。

**方案**: 局部网格的语义节点("走廊口""右转处") → 拓扑图。不需要全局 SLAM, 只记关键决策点。

**判断**: 太远了, 占位。

### 优先级 & 并行策略

```
立即可以做 (并行):
  方向三 Step 1: 拍门照片, OpenCV Canny+Hough 离线测试
  方向一 Step 1: C3 BLE 扫描固件, 验证 RSSI 逻辑

紧接着:
  方向三 Step 2: 局部网格填充 (YOLO + OpenCV → 极坐标网格)
  方向一 Step 2: HSV 直方图 + BLE 融合, 在画面里锁定主人

后期:
  方向四 Step 1: YOLO 手持物品检测验证
  方向四 Step 2: RECEIVE 靠近接收行为
  方向二: 等需要时再说
```
