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
├── C3_USB_Controller/ # 当前 RK3566 USB CDC 用 C3 固件
│   ├── C3_USB_Controller.ino
│   └── 协议/电机/超声波依赖文件
├── communication/    # USB 通信说明与历史通信记录
│   └── README.md
├── decision/         # 视觉跟随 + 蓝牙锁主 设计文档
├── state/            # 凝视/跟踪/标定 (gaze.py 救援式凝视, tracker.py 极坐标CV-KF, calib/ 内参)
├── world/            # 世界理解 (door_v3 门检测+几何管线, fusion/ YOLO+OpenCV融合)
├── gimbal/           # 两轴舵机云台 (W1救援式级联+实测教训, 详见README)
│   ├── gimbal_follow.py
│   └── README.md
├── simulation/webots/ # 桌面图形仿真 (无需 RK3566/C3/实体底盘)
├── config/           # settings.yaml
└── models/           # (空, 模型路径引用)

## 通信: RK3566 USB 直连 (当前方案)

C3 USB-C → KickPi USB-A，使用 CDC ACM 虚拟串口，通常为 `/dev/ttyACM0`。USB 同时提供供电和通信。
当前不使用 RK3566 板载 UART 或 30-pin 排针 GPIO，因为该开发板的串口/引脚连接不便于和 ESP32-C3 建立稳定物理链路。也不需要 CH340、CP2102、WiFi 或额外 UART 转换器。
踩坑顺序: RK UART无TX → CH340 OHCI崩溃 → CP2102崩溃 → AP6255 ESP32不兼容 → USB直连✅

### ⚠️ 编译必加 CDCOnBoot=cdc

ESP32-C3 的 `Serial` 默认走硬件 UART0，必须显式开 USB CDC：

```bash
arduino-cli compile --fqbn "esp32:esp32:esp32c3:CDCOnBoot=cdc" shadow_carrier_on_rockchip/C3_USB_Controller
```

不加 → C3 不会通过 USB CDC 出现预期的通信串口，KickPi 无法正常通过 USB 发送命令。

当前应编译 `C3_USB_Controller/`。`communication/` 中的同名 `.ino` 和依赖文件是早期重复工程，不是 RK 路径的主 C3 工程。

## 开机自启

| 服务 | 端口 | 功能 |
|------|------|------|
| ap-hotspot | WiFi | `ShadowCarrier-RK` / `shadow123456`, NM已排除wlan0 |
| rk-control | 80 | 网页遥控 + USB CDC→C3 |
| video-stream | 8080 | YOLO检测 + MJPEG推流 |
| camera-usb-fix | — | 保持相机 USB 常供电(禁用 autosuspend), 见 `deploy/camera-usb-fix/` |

### 一键健康自检（demo 前每次开机跑）

`scripts/health_check.sh` 汇总关键健康指标：相机节点/USB常供电 · `fps`/`age_ms`/`restarts`/`cam_reopens` · `yolo_daemon` 存活 · dnsmasq CPU/绑定 · 热点(hostapd/wlan0/手机lease) · IMU · C3 串口 · 系统服务。

```bash
bash scripts/health_check.sh                 # 单次快照, 全绿 exit 0
bash scripts/health_check.sh --watch 600 20  # 长跑 10min(每 20s 采样), demo 前排障用
```

判定红线：`age_ms < 200`、dnsmasq CPU 个位数%、`restarts` 不增长、`/dev/camera0` 不悬空。任一 FAIL 先查 `dmesg | grep 0c45`（相机 USB）与 `/dev/shm/yolo_daemon.log`（daemon 诊断）。

### 相机节点固定 + 禁用 USB autosuspend（防取流被周期性打断）

```bash
sudo cp configs/99-shadowcarrier-camera.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb --subsystem-match=video4linux
```

## 当前已知小问题与边界

以下内容是当前版本的已知限制，记录用于排查和后续收敛，不代表已经修复：

1. **C3 工程存在两份副本。** 当前 RK3566 USB 路径应使用 `C3_USB_Controller/`；`communication/` 是早期重复工程，不支持最新的 `DIFF` 差速命令，且目录名与 `.ino` 主文件名不一致，不能直接按 Arduino CLI 的目录规则编译。
2. **USB 串口恢复：枚举卡死已有看门狗，掉电型仍需电源侧。** 10-03 起 `deploy/c3-watchdog/` 每 3s 检查，异常连续 2 次自动 `usbreset`（已实测恢复枚举卡死）；但 **C3 完全掉电消失**这类软件救不了，仍需独立供电/加电容。
3. **串口健康检查还不完整。** RK 后台线程会排空 C3 的 USB CDC 输出，避免 CDC 缓冲区堵塞，但会丢弃 `PONG` 等诊断内容。`/ping` 目前只能说明文件描述符存在，不能证明命令已经得到 C3 响应。
4. **视觉管线依赖 RK 本机资产。** `video_stream_v7.py` 使用 RK 上的绝对路径启动 `yolo_daemon` 和 `.rknn` 模型；仓库本身不包含编译产物和模型，换一块 RK 板不能直接运行。
5. **原生 C++ 感知模块仍是占位实现。** `perception/camera.cpp` 和 `perception/yolov8.cpp` 的 V4L2/NPU 接口仍保留 TODO；当前实际运行的是 Python 推流脚本加外部 YOLO daemon。
6. **配置文件与实际运行入口尚未统一。** `config/settings.yaml` 仍保留 `/dev/ttyUSB0` 和 `binary` 协议字段，但当前 `rk_control.py` 使用 `/dev/ttyACM0` 和 ASCII 文本命令，而且没有读取这份 YAML。
7. **跟随控制目前主要使用横向位置。** `TARGET_BBOX_H`、`DIST_DEADBAND`、`MIN_BBOX_H`、`FWD_SPD` 等参数在当前控制循环中没有形成完整的距离控制闭环；控制周期和注释中的研究参数也仍在迭代。**10-03 仿真/真机观察验证：恒速跟随在主人静止时会持续顶上去（无距离保持），主人消失后仍满速前进**——需加"bbox 宽度/面积超阈值→STOP"兜底，或引入 MPU 航向/里程闭环。
8. **超声波保护有强制解除路径。** 无回波累计或障碍锁定超过最大时长后，传感器状态会被强制清除，目的是防止永久锁死，但这不等于确认前方安全。测试时仍应保留人工 STOP 入口，并让车轮离地。
9. **部署地址和引脚说明仍有局部不一致。** RK 控制页面里存在固定视频地址，云台脚本的注释与实际 GPIO4_A6/A7 配置也需要以当前 RK 板卡接线为准。
10. **USB 相机 autosuspend 会周期性打断取流（10-14 已修）。** 相机（Sonix `0c45:6366`）默认 `power/control=auto`（autosuspend 2s），suspend/resume 时会 **re-enumerate**（`dmesg`: `usb 7-1 disconnect/reconnect`，节点 `video10/11/12` 漂移，实测约每 5 分钟一次）。旧版 `video_stream_v7.py` 的 `VideoCapture` 持有已删除的 `/dev/videoN` → `cap.read()` 永久阻塞 → producer 停发帧 → daemon 静默 → watchdog 空转重启（`restarts` 暴涨 + `<defunct>` 僵尸）→ 检测永久失效。**修复**：`configs/99-shadowcarrier-camera.rules` 增加 `ATTR{power/control}="on"` 禁用 autosuspend；`video_stream_v7.py` v7.3 增加 `camera_watchdog`（无新帧 >2.5s 自动 release+reopen，兼容节点漂移）、daemon 子进程回收（消僵尸）、daemon stderr 落盘 `/dev/shm/yolo_daemon.log`、心跳超时仅在输入流动时重启。故障复发时先查 `dmesg | grep 0c45` 与线材/供电。
11. **dnsmasq 曾 99.7% CPU（DNS 自环，10-14 已修）。** `systemd-resolved` 已 disabled，但 `/etc/resolv.conf` 仍为 `nameserver 127.0.0.53`；dnsmasq（`interface=wlan0` → 通配绑定 `*:53`）读 `resolv.conf` 把 `127.0.0.53` 当上游，查询又被自己的通配监听接住 → 转发给自己形成死循环。**修复（最小改动）**：`/etc/dnsmasq.conf`（见 `configs/dnsmasq.conf`）加 `no-resolv` + 显式上游（`223.5.5.5`/`114.114.114.114`），从根上消除自环；DHCP 段一字未动，热点/DHCP/手机在场信号与修复前完全一致。

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

### 4. 电池/扭矩预算触底 (demo 运行硬前提)
实测：架空时双轮能转但需"外扰一下"才持续；**地面怎么扰都不动**；负重仅 +20g（排除重量）；2S 电池实测 7.6V（约 40~50% 电量）。
单侧分测左右轮都能独立转 ⇒ 电气/驱动/信号正常，**问题在扭矩预算临界**（电量↓ × 摩擦↑ 叠加）。
判读与处置（详见 `gimbal/README.md` 与 EXCHANGE 10-04）：① 满电 8.4V 复测 + 负载下量电压（跌>0.5~1V = 电池/线材）② TB6612 VM 并 470~1000µF ③ 电源修好后统一复标速度常量（`BASE_SPD 100→140~160`、`MIN_SPD 60→80~90` 离开固件死区、`ARC_SLOW 70→90`）。
⚠️ **demo 排练/展示前务必满电 + 带备用电池**。

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
| 手动舵机 | ✅ (C3直驱+网页拖拽) |
| 视觉认主 v0 (HSV+体态模板+BLE/热点双路心跳) | ✅ 实测通过; 机主原则 L0-L3 定稿(失配=丢主, 不跟路人) |
| 云台凝视 W1 (救援式级联+定向扩展搜索) | ✅ 实机 A/B 验证(连续pan为扰动源→重构); 详见 gimbal/README.md |
| 相机内参标定 (旋转扫描自标定 fx=508) + 方位解码 (0.08°) | ✅ state/calib/ |
| 极坐标世界网格 grid.json v1 (PnP标定 + 地面复标@112, 生产测距≤2%) | ✅ 已入仓 `world/fusion/`(含 sector_scores HIDE 选点, A3.1 清) |
| YOLO-World 开放词汇慢语义环 (v2s INT8 板端NPU) | ✅ 08-29 跑通(沙箱runtime), 测速/嵌入缓存待做 |
| HRI 行为状态机 (FOLLOW/WAIT/HIDE/RECEIVE/YIELD) | ✅ A1.1 真机通过(零误报)+A1.2 主人框消费端就绪 → A1.3 动作层; pose offering 归因后留 Phase C |
| 蓝牙锁主 | 🔧 BLE心跳就绪(手环9已打通), 视觉认主v0实测通过, 详见 decision/README.md 认主章节 |
| 地面复标@112 + 俯仰悬案裁定 (生产测距 ≤2%) | ✅ 10-03, world_lab/fusion/calib_prod.json (Tier1b 已知距离实物法) |
| A3.5 帧时间戳 (ts/t_mono/age_ms, /api/detections 纯增量) | ✅ 10-03, 解锁 tracker M2 时间对齐与 age 感知 |
| 滚动地图 (grid.json `map{doors,free_sectors}`, TTL 30s) | ✅ 10-03, HIDE 零延迟主路径; 扫掠降兜底 |
| Webots 数字孪生仿真台 (离线端到端, 意图驱动) | ✅ 10-03, 全场景自动测试跑通 (Windows 有 GPU 环境) |
| C3 USB 看门狗 (免物理插拔恢复枚举卡死) | ✅ deploy/c3-watchdog/, 有界诚实(掉电型仍需电源侧) |
| 舵机安全加固 (指令限位 15–165° + 步进斜坡) | ✅ 10-03 事故后全链路加固 6e6e809 |
| IMU 航向保持 (MPU6500 兼容片 @ KickPi I2C3, 50Hz) | ✅ 10-04; 直行段闭环纠偏, 详见 gimbal/README.md |
| /turn 转身到角度 (IMU 闭环, 到位误差<3°) | ✅ 代码就绪 49e8ae1; 真机验证等电池/扭矩修复 |
| sector_scores HIDE 选点 (交互亲和度打分, 纯增量) | ✅ 10-04, 世界→HRI 消费端已接 |
| pose offering 验证 → 归因"零误报成立/召回不稳" | ⏸ 留 Phase C (单目2D关键点无深度, 非调参可解) |

## 今日踩坑 (2026-08-07)

### CDCOnBoot=cdc —— 今天最大的坑
ESP32-C3 的 `Serial` 默认走硬件 UART0（C3 Super Mini 上没接）。不加 `CDCOnBoot=cdc` → C3 完全沉默 → KickPi `os.write()` 阻塞 → 双方死锁。**所有后续调试都被这个参数误导了数小时**，每次重编译都漏了它。
教训: arduino-cli 编译 ESP32-C3 时永远带 `--fqbn "esp32:esp32:esp32c3:CDCOnBoot=cdc"`。

### USB CDC 死锁
C3 的 Serial print 输出填满 CDC TX buffer → C3 loop() 阻塞 → C3 不读命令 → KickPi TX buffer 也满 → `os.write()` 阻塞。
修复: C3 固件保留必要打印(诊断用) + rk_control.py 加 uart_reader 后台线程持续排空。

### Python f-string 里的 JS 花括号
JS 的 `{}` 在 Python f-string 里必须写成 `{{}}`。忘记双写 → SyntaxError → 服务挂了。

## Webots 图形仿真

可在 macOS 或 Windows 桌面打开 `simulation/webots/worlds/shadow_carrier.wbt`，不需要 RK3566、C3 或实体底盘。仿真包含差速底盘、云台、相机识别框、超声波、障碍物和可移动目标，并复用当前跟随控制器与 HRI 状态机。运行方式、按键和仿真边界见 [`simulation/webots/README.md`](simulation/webots/README.md)。

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

### 为什么坦克转向不行（结论的适用范围）

C3 的 MOVE L/R 是原地转（左轮后+右轮前）。即使最慢速度 70，200ms 脉冲也会转过
一大截 → 人从左边跑右边 → 触发反向转 → 来回摇摆 → 丢人。

三轮迭代:
- v1: 固定速度开关控制 → 疯狂摇摆
- v2: 定时脉冲 + 方向滞回 → 一步一停, 跌跌撞撞
- v3: **差速弧线** → 不停车, 边前进边微调方向 ✅

> ⚠️ **适用范围补注 (2026-10-16)**：当年否掉的是"**拿原地转替代弧线去前进跟人**"——原地转
> v=0，插在前进里就成了"直行-转-直行-转"，永远在挪步、没有连续前进。**不是"原地转本身不行"**。
> 当年摇摆还叠加了两个具体根因，今天都不成立：
> ① 开环 200ms 定速脉冲 + 没有角速度反馈 → 一步冲过头；现在有 IMU(50Hz) 与 KF 的 θ̇；
> ② 分不清"人在横移"与"我在转" → 现在用 `θ̇ + IMU 角速度` 做运动归因。
> 所以 v5 把原地转作为**独立状态**补了回来：只在"人在侧后方 / 正在出画 / 已丢失"时调用，
> 退出即回弧线，**两者不做连续混合**（前进跟人仍然只有弧线一种形态）。

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

## 跟人控制 v5: 底盘转向通道 (2026-10-16, 云台冻结期)

**要解的问题**：云台一冻结，相机就与车体固连，水平视场只有 ±32°（FX=508.2 / 640）。
主人从车正前走到正后（demo 里必然出现的"擦身"），一过 ±32° 出画就**没有 YOLO 人框**，
而旧逻辑只有"后退 1.2s → 停车等"，**没有任何转向搜索** → 必然跟丢。

**做法**（`scripts/follow_controller.py`，总开关 `TURN_ENABLED`，逐步启用可回退）：

| 相 | 触发 | 行为 |
|---|---|---|
| TRACK | 默认 | 弧线跟随（与 v3 逐字节一致） |
| TURN | E1 预测式：`TTE=(30°−\|θ\|)/\|ω_世界\| < 0.35s` 且 `\|ω_世界\|>40°/s` 连续 2 帧<br>E2 兜底式：`\|θ\|≥30°` 持续 0.3s | `MOVE L/R 110` 原地转 |
| BLIND | 转向中丢失强检测 | 有界盲转：纯预测 60°、弱线索在 150°（防"按陈旧 θ̇ 转过头"） |
| SEARCH | 盲转额度用尽 | 朝出画侧 90° 一档 → 停 0.5s 采两帧 → 累计 180° 后原地等待 |

- **归因是核心**：`ω_世界 = θ̇(KF) + IMU 角速度`，区分"人在往外走"与"我在转"；自身大幅
  旋转时不判 E1（θ̇ 会滞后 1-2 拍，会把自身旋转误算成人）。
- **退出**：`|θ|≤15°`（与入口 30° 构成 2:1 迟滞）+ 最小驻留 0.25s；方向反转看门狗
  （5s 内 >3 次 → 强制回弧线，即 §5.4.6 的"一键回退 DIFF"）。
- **授权分级（机主原则第0层不放宽）**：强证据（YOLO+模板）才能**发起**转向与放行前进；
  弱线索（被层0拦下的低置信/贴底框、认主时派生的"下装"颜色模板）**只能维持原地转** ——
  不进候选池、不写 owner.json、不进 KF、不放行前进。"擦身时画面里只剩腿、YOLO 出不了人"
  的工况就靠它继续转而不是停下。
- **复用的既有资产**：`state/decoder` 出 θ（去畸变）、`state/tracker` 的 PolarCVKF 出 θ̇
  （此前只有假数据自测在用）。
- **安全**：超声波受阻否决、原地转不平移、单次转向有角度/时长预算、扫描不做 360° 持续旋转。

**验收与标定**：
- 离线回归：`scripts/test_turn_pivot.py`（10 组断言，含"正常跟走原地转次数 = 0"护栏）、
  `scripts/test_weak_cue.py`；原有 `test_follow_dt.py` 不变仍绿（关闭开关时行为零变化）。
- 仿真：`simulation/webots/run_passby.sh`（擦身 A/B：转向通道 开 vs 关）、
  场景 `passby`；仿真相机补了水平 FOV 裁剪（`_project_box`）与 `SIM_GT_TURN` 开关，
  不再用 ground-truth 瞬移航向冒充掉头。
- 实机：`scripts/pivot_calib.py`（PWM→°/s 实测、IMU 符号核对、θ 左右符号人工核对）。

**衍生发现（2026-10-16，仿真心跳检查逼出来的）**：补上水平 FOV 裁剪后，`baseline` 剧本的
**递物交接（RECEIVE）直接挂**——车停在 HIDE 点后，主人是从**侧方 ~1.4m（车体左 76°）**过来
递东西的，云台冻结下这根本不在 ±32° 内 → 永远不检测 → RECEIVE 起不来。旧仿真的绿是
"无限视场"假框喂出来的。修法：给转向通道加了 **停驻域 `fc.tick(turn_only=True)`**
（HIDE/WAIT 车已停好时只允许"人从侧方过来→有界转头接人"，不平移、不扫 180°），
仿真在 WAIT/HIDE 停驻时调它。**真机同样需要这条**：要么车转头接人，要么主人绕到车正前方递物。

### 2026-10-19 审查修复（离线，带回归断言）

| 修复 | 依据（实测） | 落点 |
|---|---|---|
| **框高突变门会永久冻结 EMA** | `H_CHANGE_LIMIT=0.3` 的跳变门在高度**持续**变化时（人走近 300→450）永远满足条件 → `scx/sh` 再也不更新，车保持旧方向。改为"连续 3 拍确认 → 重新播种" | `follow_controller.tick()` + `H_CHANGE_CONFIRM_N` |
| **退出原地转用冻结的 KF θ** | 174°/s 下 KF 卡方门整帧拒绝 → `_theta` 冻在 31°，真值已回 10° → 永不退出（只能烧完 180° 预算）。改为与 E1/E2 一致：**决策用原始量测** | `_steer` 的 TURN 退出判据 + `_exit_sign` |
| **进入转向后下一拍就能翻向** | `_enter_turn` 把 `_last_dir_t` 归零（= 驻留早已满足）→ 实测 0.1s 一翻。改为"驻留从进入时刻起算" | `_enter_turn` |
| **近距 off-axis 无保护** | `BASE_SPD=220` 制动距离 ≈0.9m，而框高在 ~2.2m 内饱和（1m 与 2m 同高）、脚底测距在 <1.58m 已出画 → 11°–30° 的人既不停也不减速 | ① **速度门**：`|offset|` 越大越慢（11° 内不生效，30° 保留 55%）；② **单目地面测距**（`state/range_from_box.py`，1.6–6m）接 `dist_fn`，任意方位角都能用真实距离急停/减速。
> 余量精调（制动模型推导 `NEAR_STOP_M`、载重启动阈值标定）**机主裁定封存到 V2**（V1 只要求能动起来；V2 有编码器闭环/雷达 `range_m` 才谈余量）。见 `docs/DEFERRED_WORK.md` §3 |
| **速度护栏被放宽成失效** | `test_follow_dt` 曾把 `max(vals)<=160` 改成 `<=255`（等于满量程）→ 改为有意义的包络（外轮≤BASE、内轮≥MIN、差速≤120）+ 速度门/近距门分工断言 | `test_follow_dt.py` |
| **两条新胶水代码无测试** | `_manual_forward` / `_hri_parked` 都是新增但无断言 | 注入点 + `test_control_glue.py`（含 ±180 穿越、文件过期/坏 JSON、近距测距边界） |
| **常量与转速脱钩** | `PIVOT_PWM` 110→190 后，角度预算/扫描节奏仍按旧速率设 | `PIVOT_RATE_DPS=174` 提成常量；`test_turn_pivot` 与生产同口径；`pivot_calib` 扩成一次标定四件事（PWM→°/s、弧线差速、`v_at_spd_max`、IMU/θ 符号）并输出 `calib.json` |

## 下一步: MVP → Demo 的四个方向

Demo 场景: 跟到饮水机 → 躲开门边等人流 → 跟到冰箱 → 接果冻 → 跟回去

```
┌─────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐
│ 方向一   │    │ 方向三   │    │ 方向四   │    │ 方向二   │
│ 锁主     │    │ 世界理解  │    │ 物体接收  │    │ 云台坐标  │
│ 🟢 基本完成│    │ 🟢 大步推进│    │ 🔴 demo  │    │ 🟢 提前交付│
│ (待对抗   │    │ grid v1+ │    │ 关键:    │    │ 凝视W1上线│
│  验证)   │    │ YW跑通    │    │ HRI实机  │    │ 坐标解码✅ │
└─────────┘    └─────────┘    └─────────┘    └─────────┘
```

> 进度速览 (2026-08-29): 四线中三线能力已达 demo 级; 当前唯一红灯 = HRI 实机联调
> (C1 持物实测 + 动作层对接 + grid.json 消费)。全栈联测(认主+凝视+弧线+超声)为下一里程碑。

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

> 📚 相关论文见文末[论文雷达](#论文雷达-2026-08-24)①②③

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

#### HIDE 选点语义 (2026-08-29 定稿, [HRI]线)

"安全" ≠ 离人远, 而是 **不打扰但可被找到** (服务员侧身让路式)。选点打分
以**交互亲和度**为核心——躲开人类高频接触的地方:

| 语义 | 交互亲和度 | 对选点 |
|------|-----------|--------|
| 门 / 走道口 | 极高 (穿行) | **强惩罚**, 含其进出锥方向 |
| 冰箱 / 饮水机 / 水槽 | 高 (被靠近) | 强惩罚其前方扇区 |
| 两热点连线 | 高 (路径) | 惩罚 = "路中间"的形式化 |
| 墙角 / 背墙位 | 低 | **加成** (背后不可接近) |

注: 门由几何环 (door_v3) 与 YOLO-World 双路供给, "door" 查询词待调优;
交互亲和度惩罚优先于"门加成"式地标吸引——躲的是人流, 不是贴地标。

#### 触发时序 (滚动地图为主路径)

```
跟随时: 近环占用+物体方位 顺带入滚动地图 (轻量, 不跑门配对等重管线)
WAIT(5s) → 查地图有候选 → HIDE 直接走 (10~15s 静止即触发, 无盲扫窗口)
                        → 地图空/过期 → 兜底扫掠 (7s, 每帧可中断)
主人恢复移动/消失>2s/快速靠近/递物 → 任一命中立即中断并 /gimbal 归位
码盘+MPU6050 到位后: 瞥视间航位推算维持地图 + 精准回位 + 路径记忆(拓扑)
```

> 📚 相关论文见文末[论文雷达](#论文雷达-2026-08-24)④⑤⑥

#### 参数与标定 (换相机/改高度不用改逻辑)

`scripts/hri_params.json` 是 HRI 唯一调参入口, 分三层:

| 层 | 内容 | 换相机高度时 |
|---|---|---|
| **物理量 (优先)** | 距离/速度用肩宽反推 `dist = fx·shoulder_width_m / w_px` (姿态免疫); 弯腰用宽高比 (比值) | 重跑 `auto_calib_pose.py` 更新 `camera` 段 |
| **像素降级** | 标定缺失时自动回落 (日志 `mode=pixel`) | 无需改 |
| **比值类** | 弯腰比/基线系数等无量纲量 | 天然免疫 |

已知边界: 人像**裁切**时姿态信号冻结(不可信), 距离闸由裁切本身兜底。
"物理量优先"同时修掉了三个耦合 bug: 高度反推距离会把弯腰误判为远离/走近;
裁切会污染宽高比基线; 接近中会被误判"停滞"。详见 `scripts/hri_state.py` 注释。

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

> 📚 相关论文见文末[论文雷达](#论文雷达-2026-08-24)⑦

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

---

## 论文雷达 (2026-08-24)

> 按四方向整理的文献调研, 按"对本项目对胃口程度"排序。原则: **0.8 TOPS RK3566 + 单目 USB 摄像头 + 无训练设施** 是硬约束, 需要 GPU 训练/推理的方法只取思想不取工程。

### ① Follow-Bench — 跟人任务的统一基准 ⭐ 全局必读
- **出处**: Ye et al., arXiv:2509.10796, 代码/数据全开源 (follow-bench.github.io)
- **内容**: 首个 RPF (Robot Person Following) 端到端综述+仿真基准。复现了 SFM/MPC/DWA 等 6 种规划器, 强调两大需求: (i) 自然持续跟随目标 (ii) 主动避障与行人社会性避让; 并给出安全-舒适权衡的量化结论。
- **对本项目的用法**:
  - 跟随控制 v3 (差速弧线) 的对照系——MPC w/ Traj. 在动态环境表现最好的结论, 对应我们"丢失惯性保持+比例变速"的未来升级方向
  - demo 评审指标的来源: 目标可见率 (TSR)、轨迹平滑度、人的舒适度
  - 实机就是差速底盘, 与我们硬件形态一致

### ② EscortE — 伴随场景在线动作检测 ⭐ HRI 状态机的直接参照
- **出处**: Mondal et al., arXiv:2506.23573
- **内容**: 护送机器人场景下, ViT 嵌入联合做 person Re-ID + 动作识别, 输出 following / lagging / stopping 三态, 据此调速。
- **对本项目的用法**: 与 follow_controller.py 里 lost/wait_owner 状态判断一一对应; "主人停下→我等待"正是 WAIT 态的学术表述。其 ReID 分支可对照我们的 HSV+体态比方案评估是否需要升级。

### ③ SDA — 社会动力学自适应跟人 (ICLR 2025) ⭐ 叙事框架
- **出处**: Scofano et al., ICLR 2025, Habitat 3.0 平台 SotA, 代码开源
- **内容**: 从机器人自身 state-action 历史推断社会动力学 (不需要人类轨迹特权信息), 两阶段 RL 学出 Find → Seek → Follow → Lost 四种行为模式的隐空间表征。
- **对本项目的用法**: 给 S-W-R (Separation–Wait–Reunion) 状态机提供学术化的行为分段依据和可视化方法 (t-SNE 行为聚类图可直接模仿进作品集)。核心卖点"只用机器人自身历史"恰好符合我们的边缘约束。

### ④ 轻量自监督单目深度 @ RK3588S 67FPS ⭐ 世界模型的落地路径
- **出处**: Yang et al., J. Measurement Science and Instrumentation, 2026, 17(2): 278-296
- **内容**: 3M 参数 / 3.5 GFLOPs 的自监督单目深度网络 (大核卷积分解+跨尺度特征差分融合), KITTI AbsRel=0.105, 在 **Rockchip RK3588S NPU 上混合量化跑到 67 FPS**。
- **对本项目的用法**: **本项目最重要的外部证据**——证明"NPU 跑深度→地面分割→自由空间"在 Rockchip 上是现实路径, 可替代 Canny+Hough 启发式做地面可通行性判断, 且比手写几何规则鲁棒一个量级。RK3566 算力约为 RK3588S 的零头, 需降到 ~160×120 输入实测帧率, 但部署管线 (rknn-toolkit2 转 RKNN → C++ 推理) 与我们 yolo_daemon 完全同构。

### ⑤ Adap-RPF — 遮挡下的自适应跟人规划
- **出处**: Situ et al., arXiv:2510.11308, 实机验证
- **内容**: 目标为中心的社会空间内密集采样候选跟随点, 多目标代价函数 (可见性/遮挡/舒适/平滑) 选点, 预测感知 MPPI 控制器跟踪。
- **对本项目的用法**: "丢失惯性保持"的系统性升级蓝图。demo 后如果要做复杂人流场景, 候选点采样思想可以直接嫁接到我们的极坐标网格上 (网格格子=候选点)。

### ⑥ GUIDER — 运动证据 × 占据栅格的概率意图推理
- **出处**: Contreras et al., J. Intelligent & Robotic Systems 112:36, 2026 (open access)
- **内容**: Synergy Map 把人体运动证据与占据栅格融合, 无需预定义目标即可排序"人可能要交互的区域"; 双阶段 (导航期+操作期) 贝叶斯更新。
- **对本项目的用法**: 我们"极坐标网格 + 主人意图信号"融合设计的概率版原型。RECEIVE 触发条件 (面向+持物+靠近) 可以借它的贝叶斯打分框架从硬阈值变成软概率。

### ⑦ Event-Driven Proactive Assistance — 事件驱动的主动辅助
- **出处**: arXiv:2603.23950, 真实桌面协作平台验证
- **内容**: 提出"从请求驱动到事件驱动"范式: 不等指令, 监测人-物交互造成的场景状态跃迁 (事件前后快照), 用 VLM 推断任务级目标并决定是否介入、如何辅助。
- **对本项目的用法**: RECEIVE 行为的理论骨架。"检测到主人手持物体缓近 = 一次事件触发"——把三信号硬编码条件包装成事件监测机制, 是答辩叙事里"隐式交互"的最佳注脚。

### ⑧ DepthDrive — 单目测距的开源实现参考 (GitHub)
- **出处**: github.com/Nawfel-9/DepthDrive (Jetson/TensorRT)
- **内容**: YOLO分割+单目深度融合输出米制距离速度; 关键技巧: **画面里有 person 时用人体真实身高当标尺动态标定像素→米换算** (针孔模型)。
- **对本项目的用法**: 方向二 (云台坐标解码) 的廉价实现路线: bbox 高度 + 已知身高先验 + FOV 粗标定 → 米制距离, 不需要棋盘格全套标定。深度模型跑不动时这是零成本替代。

### ⚠️ 看着诱人但工程上不碰的
NavWM / UniNav / WNM-3D 等 diffusion 世界模型 (arXiv 2606.24101 / 2608.03244 / 2608.07267): 需要 GPU 推理+大规模训练, 0.8 TOPS 上无部署路径。**引用其思想** (latent world token、几何条件化、世界模型做监督而非直接策略) 撑论文门面即可; 工程上"几何快环+极坐标网格"仍是对的路。
