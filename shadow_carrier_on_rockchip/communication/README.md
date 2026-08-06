# Communication — KickPi ↔ C3 通信

> **最终方案：WiFi TCP。** RK3566 板载 UART 无 TX 信号、USB-TTL 模块兼容性问题 → 走 KickPi 自开热点 + C3 WiFi 直连，TCP:8888 收发 ASCII 命令。协议不变。

## 架构

```
KickPi (hostapd 热点 192.168.4.1)
   │  TCP client → 192.168.4.2:8888
   ↓
C3 (WiFi STA 连热点)
   │  解析 MOVE/STOP/PING
   ↓
TB6612 → 电机
```

## C3 WiFi 固件

文件: `C3_WiFi_Controller/C3_WiFi_Controller.ino`（或根目录 `C3_WiFi_Controller.ino`）

- 连 KickPi 热点 `ShadowCarrier-RK` / `shadow123456`
- TCP Server 端口 8888
- 接收 ASCII 命令，复用原 CommandParser/MotorDriver/UltrasonicSensor
- 保留 450ms 超时 / 速度斜坡 / 超声波避障

## RK3566 板载 UART 调试记录

### 问题: TX 无信号
三个板载 UART (ttyS5/S7/S9) 时钟正常、pinmux 正确、DTS okay，但 TX 物理引脚不输出电压。UART7→UART9 loopback 测试 0 字节。

### CH340/CP2102 USB-TTL 尝试
- CH340: 检测正常、自环通，但接 C3 后无信号，且触发 RK3566 USB 总线崩溃（内核 OHCI bug）
- CP2102: 同崩溃
- 结论: RK3566 BSP 5.10.160 内核的 USB 主机驱动有已知缺陷，无法稳定驱动 USB-TTL 模块

### 最终方案: WiFi TCP
C3 连 KickPi 热点，TCP socket 通信。延迟 2-5ms，远小于 C3 450ms 超时。

## 协议
协议完全不变——ASCII `MOVE F 180\r\n` / `STOP\r\n` / `PING\r\n`，115200→TCP。C3 原有指令集和电机驱动完全保留。

## 参考
- C3 原 UART 固件: `DistributedRobot_C3_MotionController/`
- S3 网关（已退役）: `DistributedRobot_S3_Gateway/`
- 决策设计: `../decision/README.md`
