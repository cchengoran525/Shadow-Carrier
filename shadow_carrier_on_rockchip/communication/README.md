# Communication — RK3566 ↔ C3 USB CDC 通信

> **当前方案：USB 直连。** 由于 RK3566 开发板的板载 UART/排针 GPIO 不方便和 ESP32-C3 建立稳定物理连接，当前使用 RK USB 口直连 C3，不走 RK 引脚 UART。

## 架构
```
KickPi USB-A ──USB线──> C3 USB-C
   /dev/ttyACM0           Serial (CDC ACM)
   发 MOVE/STOP           接收→解析→驱TB6612
```

## C3 固件

当前 RK 路径使用上级目录的独立工程：

```text
shadow_carrier_on_rockchip/C3_USB_Controller/
```

请使用其中的 `C3_USB_Controller.ino` 和依赖文件编译。这里的 `Serial` 指 USB CDC 通道，不是 C3 的 GPIO10/GPIO11 硬件 UART。

根目录的 `DistributedRobot_C3_MotionController/` 仍保留原来的 S3-C3 GPIO UART 固件，两条路径共用 ASCII 命令格式，但不是同一个物理通信入口。

这份目录主要保存通信说明。若要编译当前 RK3566 USB 固件，请回到上级目录使用 `C3_USB_Controller/`，不要把这里的重复 `.ino` 当作主工程。

## 踩过的坑
| # | 方案 | 结果 |
|---|------|------|
| 1 | RK板载UART ttyS5/S7/S9 | ❌ TX无物理信号 |
| 2 | CH340 USB-TTL | ❌ 自环通但接C3无信号，OHCI崩溃 |
| 3 | CP2102 USB-TTL | ❌ 同崩溃 |
| 4 | WiFi TCP (C3连热点) | ❌ AP6255 beacon ESP32-C3不可见 |
| 5 | C3 USB直连 | ✅ 即插即用 |

## 协议
ASCII `MOVE F 180\r\n` / `STOP\r\n` / `PING\r\n`。命令格式与原 S3-C3 UART 路径保持一致；当前 RK3566 使用 USB CDC 作为传输介质。
