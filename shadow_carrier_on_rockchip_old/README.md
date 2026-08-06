# Shadow Carrier on Rockchip

小车大脑主项目。运行在 KickPi (RK3566) 上，通过视觉感知+多模态融合认主并跟踪，通过串口控制 ESP32-C3 驱动底盘。

## 架构

```
shadow_carrier_on_rockchip/
├── perception/       # 感知层：摄像头+YOLO+蓝牙+颜色/姿态签名
│   ├── camera.cpp    # USB摄像头驱动、MJPG取帧
│   ├── yolov8.cpp    # NPU YOLO推理封装（引用现有C++引擎）
│   └── owner_id.py   # 三信号融合认主（蓝牙+颜色+姿态）
├── decision/         # 决策层：状态机
│   └── state_machine.cpp  # 状态切换：扫描→认主→跟踪→跟随
├── control/          # 控制层：底盘运动算法
│   ├── motor.cpp     # PID速度控制
│   └── tracking.cpp  # 卡尔曼滤波+路径跟随
├── communication/    # 通信层：与下位机交互
│   └── esp32.cpp     # UART串口协议（与ESP32-C3通信）
├── gimbal/           # 两轴舵机云台（独立模块, 追最大person）
│   ├── gimbal_follow.py  # 轮询v7 API驱动pan/tilt舵机
│   └── README.md     # 接线(GPIO4_A6/A7+5V) + 软件PWM方案
├── scripts/          # 辅助脚本
│   ├── fastyolo.py   # 快速拍照+YOLO测试
│   ├── stream.py     # MJPEG推流（待实现）
│   └── calibrate.py  # 标定/调试工具
├── models/           # 模型文件（软链接到现有位置）
│   └── yolov8.rknn -> ../../shopping_car_vision/...
├── config/           # 配置文件
│   └── settings.yaml # 主人手机MAC、阈值、PID参数等
├── main.cpp          # 主入口：启动所有线程
└── README.md
```

## 依赖：已有资产（不动不碰）

| 组件 | 路径 | 用途 |
|------|------|------|
| YOLO推理引擎 | `../shopping_car_vision/rknn_model_zoo/examples/yolov8/cpp/build/rknn_yolov8_demo` | NPU目标检测 |
| YOLO模型 | `../shopping_car_vision/rknn_model_zoo/examples/yolov8/cpp/build/model/yolov8.rknn` | 量化模型 |
| RKNN运行时 | `/usr/lib/librknnrt.so` | NPU驱动 |
| OpenCV | 系统安装 | 图像处理 |
| hcitool | 系统自带 | 蓝牙扫描 |

## 设计原则

- 不动 `shopping_car_vision` 任何文件，只引用
- 模型通过软链接或路径配置指向原位置
- C++ 做感知和控制的实时部分，Python 做逻辑和调试工具
- ESP32 通信走 UART 串口（/dev/ttyS0 或 USB CDC）

---

## 开发路线图

五个独立模块，按依赖关系和优先级排序。

### 模块依赖图

```
实体搭建 ─────────────────────────────────────────┐
  │  KickPi上电、摄像头固定、C3接线、供电            │
  │                                                │
  ├──→ Communication ──────────────┐               │
  │      UART 接线 + PING 验证      │               │
  │      "KickPi能跟C3说话了"       │               │
  │                                 │               │
  └──→ 优化 YOLO IO ───────────────┘               │
        ✅ 已完成: 3.7 → 11.4 FPS (真实)          │
        "不再眼瞎卡顿"                              │
                                    ↓               │
                              基础跟人程序           │
                              YOLO bbox → MOVE指令   │
                              "能追着人跑了"         │
                                    │               │
                                    ↓               │
                              主人锁定程序
                              BT+颜色+姿态 → 只跟主人
                              "不会跟错人了"
```

### 模块 1：实体搭建（最前置，阻塞所有后续）

**先决条件，不做一切免谈。**

| 子项 | 说明 |
|------|------|
| KickPi 供电 | 充电宝诱骗 12V？独立电池组？需持续供电方案 |
| 摄像头固定 | 视野高度、俯仰角度，USB 线缆走线 |
| C3 接线 | 3 根杜邦线：TX/RX/GND，确认 KickPi 30pin 排针上 UART 引脚位置 |
| 底盘供电链路 | C3 + TB6612 + 电机 + 电池 |
| 整机结构 | 各组件固定、重心平衡、线缆收纳 |

> 此模块与代码无关，是物理世界的事。

### 模块 2：Communication（阻塞"基础跟人"）

**实体搭好后的第一个代码任务。工作量：~50 行 Python。**

目标：KickPi 能通过 UART 跟 C3 对话，完全替代 ESP32-S3 Gateway 的角色。

| 步骤 | 内容 |
|------|------|
| 选 UART | 在 `/dev/ttyS5` / `ttyS7` / `ttyS9` 中确认物理接线 |
| 写 Bridge | 实现 `C3Bridge` 类（已在 [communication/README.md](communication/README.md) 给出参考） |
| PING 验证 | `PING` → 收到 `PONG`，确认双向通信 |
| MOVE 验证 | 发送 `MOVE F 100` → 电机转 → `STOP` → 停 |
| 速度标定 | 确定各方向实际速度曲线，为控制层积累参数 |

协议细节见 [communication/README.md](communication/README.md)。C3 固件**一行不改**。

### 模块 3：优化 YOLO IO（✅ 已完成）

**2026-08-04 达成 11.4 FPS 真实检测帧率（v7 信号量背压流水线），全程纯软件，无硬件改动。**

> ⚠️ 曾虚报 14.6 FPS——daemon stdout 被 rknn 库 printf 污染，Python 没真正等推理完成。修复后测得真实值。

| 子项 | 说明 |
|------|------|
| v1 起点 | subprocess 冷启动，0.8 FPS |
| v2 | daemon 常驻，3.7 FPS，瓶颈=eMMC磁盘I/O + PNG编码 |
| v3 改动 | ① 帧文件移入 `/dev/shm`(tmpfs内存盘) ② daemon 输出改 JPG ③ 去多余resize ④ 直接读字节推流 |
| v7 结果 | **11.4 FPS** 真实检测帧率，read_fail=0 零坏帧（无花屏），延迟有界 ~200ms，稳定运行 |
| 备选方案 | 全 C++ 管线（填 camera.cpp + yolov8.cpp 的 TODO） |

**v7 关键实现（详见 [perception/README.md](perception/README.md)）：**

```
[生产者线程] 抓帧→imencode→双缓冲(/dev/shm)→喂daemon   (信号量背压, ≤2帧在飞)
[yolo_daemon] 读→NPU推理→画框→tmp+rename原子写out.jpg    (JSON走stderr隔离)
[消费者线程]  收JSON→读out.jpg→直接字节推流               (读完整帧, read_fail=0)
```

**优化四杠杆：** ① C++daemon常驻(免冷启动) ② /dev/shm内存盘+JPG(免磁盘/PNG) ③ 双线程流水线(免串行) ④ 信号量背压(免延迟累积/坏帧)

### 模块 4：基础跟人程序（第一个闭环）

**依赖：Communication ✅ + YOLO IO ✅**

核心逻辑 ~100 行，最小可行产品——能追人就行，不区分是谁。

```
YOLO 检测 person 的 bbox
    ↓
bbox 中心 x 坐标 / 画面宽度
    ↓
┌──────────┬──────────┬──────────┐
│ 偏左 1/3  │ 居中      │ 偏右 1/3  │
│ MOVE L   │ 面积判断   │ MOVE R   │
│          │ 太小→MOVE F│          │
│          │ 太大→STOP │          │
└──────────┴──────────┴──────────┘
```

| 状态 | 触发条件 | 输出指令 |
|------|----------|----------|
| 跟随前进 | 人在画面中央 + bbox 面积 < 阈值 | `MOVE F <speed>` |
| 左转修正 | 人偏左 | `MOVE L <speed>` |
| 右转修正 | 人偏右 | `MOVE R <speed>` |
| 停车等待 | bbox 面积 > 阈值（太近） | `STOP` |
| 丢失搜索 | 未检测到 person | 原地旋转搜索 `MOVE L 100` |

> Speed 可以按误差大小做比例控制（P 控制）：偏差大 → 速度快；偏差小 → 速度慢。先跑通开关量，再上 PID。

### 模块 5：主人锁定程序（最后，核心差异化）

**依赖：基础跟人稳定跑通。**

多模态信号融合，解决"一群人中只跟主人"的问题。

| 信号层 | 方法 | 作用 |
|--------|------|------|
| 蓝牙 RSSI | 手机 BLE 广播，`hcitool lescan` 或 `bluepy` 读 RSSI | ID token：谁靠近听谁的。RSSI 最强 = 主人 |
| HSV 颜色直方图 | OpenCV 计算 bbox 内 HSV 直方图，与主人模板比较 | 防丢锁：持续追踪时靠颜色确认还是同一个人 |
| bbox 姿态比例 | 宽高比 + 面积 | 辅助信号：排除明显不对的检测（蹲下/小孩/物体）|

**融合策略：**

```
开机 → 蓝牙扫描 → 捕获主人 MAC + 最强 RSSI 对应的 bbox
    → 提取 HSV 模板（主人衣服颜色特征）
    → 持续跟踪：bbox 匹配 = 颜色相似度 × 姿态相似度
    → 跟丢了 → 重新扫描蓝牙 RSSI 找回
```

设计原则：
- 每天开机白板重置，蓝牙 RSSI 作为当天的 ID token
- 颜色模板在光照变化时逐步更新（指数移动平均）
- 不引入神经网络，纯传统 CV + 信号处理即可
- 人脸识别留作可选增强（MobileFaceNet），不阻塞核心流程

---

## 推荐执行顺序

| 阶段 | 做什么 | 谁做 | 阻塞关系 |
|------|--------|------|----------|
| ✅ | 模块 3：优化 YOLO IO（11.4 FPS 真实） | 已交付 | 无 |
| **现在** | 模块 1：实体搭建 | 物理（焊接/接线/固定） | 无 |
| **实体OK后** | 模块 2：Communication 串口调通 | 代码 | 等实体 |
| **接着** | 模块 4：基础跟人闭环 | 代码 | 等 Comm + YOLO |
| **最后** | 模块 5：主人锁定 | 代码 | 等基础跟人稳定 |

---

## 参考

- 小车下位机仓库：https://github.com/cchengoran525/Shadow-Carrier
- 通信协议细节：[communication/README.md](communication/README.md)
- 感知层细节：[perception/README.md](perception/README.md)
