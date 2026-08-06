# Communication — KickPi ↔ C3 小脑串口协议

## 架构定位

```
┌─────────────────────────────────────────────────────┐
│  KickPi RK3566 (大脑 / Cerebrum)                      │
│  - 摄像头 + YOLO NPU 感知                             │
│  - 主人识别 (蓝牙 + 颜色 + 姿态融合)                    │
│  - 追踪决策 → 运动指令                                 │
│  - UART 发送 ASCII 协议命令                             │
└──────────────────┬──────────────────────────────────┘
                   │ UART 115200 8N1
                   │ TX ────────────→ C3 RX (GPIO10)
                   │ RX ←──────────── C3 TX (GPIO11)
                   │ GND ──────────── C3 GND
┌──────────────────┴──────────────────────────────────┐
│  ESP32-C3 (小脑 / Cerebellum) — 保留不动              │
│  - 接收 UART ASCII 命令                                │
│  - 解析协议 → 驱动 TB6612 双电机                       │
│  - 差分驱动底盘                                        │
└─────────────────────────────────────────────────────┘
```

KickPi **完全替代** ESP32-S3 Gateway。对 C3 来说，发命令的从 S3 换成了 KickPi，协议不变。

---

## 物理接线

### 原始 S3 ↔ C3 接线（参考，S3 将被移除）

| S3 Gateway | 方向 | C3 Motion |
|------------|------|-----------|
| GPIO1 (TX) |  →   | GPIO10 (RX) |
| GPIO2 (RX) |  ←   | GPIO11 (TX) |
| GND        |  ↔   | GND |

### 新 KickPi ↔ C3 接线（目标）

| KickPi | 方向 | C3 Motion |
|--------|------|-----------|
| UART TX (ttyS5/S7/S9) | → | GPIO10 (RX) |
| UART RX (ttyS5/S7/S9) | ← | GPIO11 (TX) |
| GND | ↔ | GND |

> **注意：** KickPi 和 C3 必须共地 (GND)。电压电平：KickPi 排针 3.3V，C3 也是 3.3V，直连安全，无需电平转换。

### KickPi 可用串口（K11C 30pin 排针）

| UART | 设备节点 | 芯片 GPIO | 状态 |
|------|----------|-----------|------|
| UART5 | `/dev/ttyS5` | GPIO3_C2(RX), GPIO3_C3(TX) | ✅ 可用 |
| UART7 | `/dev/ttyS7` | GPIO4_A2(RX), GPIO4_A3(TX) | ✅ 可用 |
| UART9 | `/dev/ttyS9` | GPIO4_A4(RX), GPIO4_A5(TX) | ✅ 可用 |

> `/dev/ttyS1` 已被板载蓝牙占用 (brcm_patchram_plus1)，不可用。

---

## 通信协议

### 物理层

| 参数 | 值 |
|------|-----|
| 波特率 | **115200** |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验位 | None |
| 流控 | **无** (No RTS/CTS) |
| 最大命令长度 | **48 字节** |

### 命令格式

**ASCII 文本，每条以换行结尾，大小写不敏感（C3 内部转大写处理）。**

| 命令 | 参数范围 | 说明 |
|------|----------|------|
| `MOVE F <speed>` | speed: 0-255 | 前进，speed=180 为默认中速 |
| `MOVE B <speed>` | speed: 0-255 | 后退，speed=180 为默认中速 |
| `MOVE L <speed>` | speed: 0-255 | 左转，speed=150 为默认 |
| `MOVE R <speed>` | speed: 0-255 | 右转，speed=150 为默认 |
| `STOP` | 无参数 | 立即停止所有电机 |
| `PING` | 无参数 | 心跳检测，C3 应回复 `PONG` |

### 命令示例

```
MOVE F 180\r\n    # 全速前进
MOVE L 100\r\n    # 慢速左转
STOP\r\n          # 急停
PING\r\n          # 心跳
```

### C3 内部解析逻辑

来源：`CommandParser.cpp`（Shadow-Carrier 仓库 v0.2）

1. 读取串口缓冲区一行（以 `\r\n` 或 `\n` 结尾）
2. Trim 首尾空白，转大写
3. 匹配 `"STOP"` → 立即停止
4. 匹配 `"PING"` → 回复 `PONG`
5. 匹配 `"MOVE <DIR> <SPEED>"`：
   - DIR = `F`/`B`/`L`/`R`，无效则返回 `Invalid`
   - SPEED = 整数 0-255，超范围则返回 `Invalid`
   - 不允许有多余 token
6. 非法命令：忽略或返回错误

### KickPi 侧 Python 实现要点

```python
import serial

class C3Bridge:
    def __init__(self, port="/dev/ttyS7", baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        # 关键：关硬件流控，跟 C3 一致
        self.ser.rtscts = False

    def _send(self, cmd: str):
        """发送原始命令字符串"""
        self.ser.write(f"{cmd}\r\n".encode())

    def move(self, direction: str, speed: int):
        """direction: F/B/L/R, speed: 0-255"""
        speed = max(0, min(255, int(speed)))
        self._send(f"MOVE {direction.upper()} {speed}")

    def stop(self):
        self._send("STOP")

    def ping(self) -> bool:
        """发送 PING，返回是否收到 PONG"""
        self.ser.reset_input_buffer()
        self._send("PING")
        resp = self.ser.readline().decode().strip()
        return resp.upper() == "PONG"

    def close(self):
        self.ser.close()
```

### C3 侧关键配置（不动）

| 参数 | 值 | 位置 |
|------|-----|------|
| 运动 UART RX | GPIO10 | Config.h |
| 运动 UART TX | GPIO11 | Config.h |
| 波特率 | 115200 | Config.h |
| 最大命令长度 | 48 | Config.h |
| TB6612 STBY | GPIO20 | Config.h |
| TB6612 AIN1/2 | GPIO4/5 | Config.h |
| TB6612 PWMA | GPIO6 | Config.h |
| TB6612 BIN1/2 | GPIO7/8 | Config.h |
| TB6612 PWMB | GPIO9 | Config.h |

---

## KickPi 替换 S3 的差异

| 维度 | 原 S3 Gateway | 新 KickPi |
|------|--------------|-----------|
| 联网方式 | WiFi AP (`ShadowCarrier-S3`) | WiFi 客户端 或 以太网 |
| 控制入口 | HTTP 网页 (`192.168.4.1`) | 本地决策引擎 或 SSH/Web |
| 指令来源 | 网页按钮 → UART | YOLO 感知 → 决策算法 → UART |
| 协议 | ASCII MOVE/STOP/PING | **完全相同，不做任何修改** |
| 心跳 | 网页触发 PING | 可选定期 PING 检测 C3 存活 |

**C3 固件一行不改。** 这就是分布式架构的好处——大脑换了，小脑无感。

---

## 集成步骤

1. **接线验证**：KickPi 任一可用 UART → C3，共地
2. **串口测试**：KickPi 上跑 `C3Bridge.ping()` 确认收到 `PONG`
3. **移动测试**：发 `MOVE F 100` 确认电机转动，发 `STOP` 确认停止
4. **感知联调**：YOLO 检测人物位置 → 决策 (偏左则 MOVE L, 居中则 MOVE F)
5. **闭环追踪**：摄像头帧 → YOLO bbox → 位置误差 → 速度指令 → C3

---

## 文件清单（本目录待实现）

| 文件 | 功能 |
|------|------|
| `uart_bridge.py` | Python 串口桥接，封装 C3 协议 |
| `uart_bridge.cpp` | C++ 版本（可选，用于纯 C++ 管线）|
| `protocol_test.py` | 独立测试脚本，验证 C3 通信 |

---

## 参考

- Shadow-Carrier 仓库：https://github.com/cchengoran525/Shadow-Carrier
- C3 固件目录：`DistributedRobot_C3_MotionController/`
- S3 固件目录（将被替换）：`DistributedRobot_S3_Gateway/`
- C3 协议定义：`CommandParser.cpp` / `Config.h` / `Protocol.h`
