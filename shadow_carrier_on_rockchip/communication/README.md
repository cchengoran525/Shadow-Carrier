# Communication — KickPi ↔ C3 通信

> **最终方案：USB 直连。** C3 USB线直连KickPi，CDC ACM 虚拟串口。零额外硬件。

## 架构
```
KickPi USB-A ──USB线──> C3 USB-C
   /dev/ttyACM0           Serial (CDC ACM)
   发 MOVE/STOP           接收→解析→驱TB6612
```

## C3 固件
`C3_USB_Controller.ino` — 与原C3固件同目录的依赖文件一起编译。

## 踩过的坑
| # | 方案 | 结果 |
|---|------|------|
| 1 | RK板载UART ttyS5/S7/S9 | ❌ TX无物理信号 |
| 2 | CH340 USB-TTL | ❌ 自环通但接C3无信号，OHCI崩溃 |
| 3 | CP2102 USB-TTL | ❌ 同崩溃 |
| 4 | WiFi TCP (C3连热点) | ❌ AP6255 beacon ESP32-C3不可见 |
| 5 | C3 USB直连 | ✅ 即插即用 |

## 协议
ASCII `MOVE F 180\r\n` / `STOP\r\n` / `PING\r\n`。不变。
