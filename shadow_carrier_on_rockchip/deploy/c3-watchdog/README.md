# C3 USB Watchdog (免物理插拔恢复 C3)

## 背景
ESP32-C3 通过 USB-C 直连 KickPi 时, 偶发"从 USB 消失 / 枚举卡死", 表现为
网页遥控还在但指令到不了 C3、云台失声、电机无响应 —— 历史上需**手动重新插拔**。
典型触发: 舵机堵转电流冲击导致掉电/复位 (见 EXCHANGE 2026-10-03 [云台] 事故分析)。

## 能力边界 (诚实说明)
| 情况 | 本看门狗 | 说明 |
|---|---|---|
| C3 仍在 USB 上, 但软链/服务异常 (枚举卡死) | ✅ 自动 `usbreset` 恢复 | 已实测通过 |
| C3 从 USB 完全消失 (掉电) | ❌ 只能记日志等待自行枚举 | 需硬件/电源侧解决(独立供电/加电容) |

## 文件
- `c3_watchdog.sh` —— 每 3s 检查 `303a:1001` 与 `/dev/c3_controller`; 连续 2 次异常则 `usbreset BBB/DDD` 复位, 失败则重启 `rk-control`
- `c3-watchdog.service` —— systemd 常驻 (Restart=always)

## 安装 (板端, 需 root)
```bash
sudo install -m 755 c3_watchdog.sh /usr/local/bin/c3_watchdog.sh
sudo cp c3-watchdog.service /etc/systemd/system/c3-watchdog.service
sudo systemctl daemon-reload
sudo systemctl enable --now c3-watchdog
```

## 验证 (安全: 仅删软链, 设备仍在)
```bash
sudo rm -f /dev/c3_controller     # 模拟异常
sleep 15
ls -l /dev/c3_controller          # 应已被 udev 重建
sudo tail /var/log/c3_watchdog.log
```
预期日志: `reset via usbreset 005/002 (5-1)` → `recovered (symlink present again)`。

## 备注
- 现有 udev 规则 `99-c3-usb.rules` 会在设备重新枚举时自动重启 `rk-control`, 与看门狗互补
- 若 C3 掉电型掉线反复出现: 优先解决电源(电机/舵机与 C3 分开供电、加电容), 而非换传输

## 为什么"原生 USB 老要重插"（2026-10-21 调研）

**不是 USB-C 接口的通病，是 ESP32-C3/S3 内置 USB Serial/JTAG（原生 USB）的已知行为：**

1. **C3 官方文档（USB Serial/JTAG Console → Sleep Mode）**：light sleep 时 APB/USB PHY 时钟被门控，**USB PHY 的 D+ 仍保持上拉** → 主机看不到"拔出"，可能**不会重新枚举**；官方给的恢复办法就是**物理拔插 USB**。
   <https://docs.espressif.com/projects/esp-idf/en/v5.2/esp32c3/api-guides/usb-serial-jtag-console.html>
2. **Espressif FAQ（USB）**：C3/S3 的 USB-Serial-JTAG 在芯片重启时会重新初始化；若固件不稳定/反复复位，主机会看到**每隔几秒连上又断开**。建议：手动进下载模式，或**先烧一版稳定固件走 UART**。
   <https://docs.espressif.com/projects/esp-faq/en/latest/software-framework/peripherals/usb.html>
3. **TinyUSB #2943（S3）**：S3 上电时 DP/DM 先被 mux 到 Serial-JTAG，固件再切到 OTG 时主机可能已锁在 JTAG；**中间加一个 USB hub 就能正常枚举**。
   <https://github.com/hathach/tinyusb/issues/2943>
4. **`ttyACM*` 是 MCU 本身**：复位/重启时端口必然消失再出现（`ttyUSB*` 的桥片则不会）。所以"复位就掉口"是原生 USB 的正常语义；问题在于**主机不一定重新枚举**。
5. 另有一个独立常见原因：**充电线（charge-only）只有 VBUS、没有 D+/D-** → LED 亮但 `lsusb` 里没有设备。换数据线即可。

## S3 重构建议（避免反复拔插）

按"最稳 → 次稳"排序：

1. **控制链路改用 USB-UART 桥**（CH340/CP2102/FT232，`ttyUSB*`）：桥片与 MCU 解耦，**MCU 复位不影响串口枚举**；原生 USB 只留给烧录/调试。这是机器人控制链路最稳的做法。
2. **S3 与 RK 之间加一个 USB hub**（TinyUSB #2943 的实证），并用**支持每端口断电的 hub + `uhubctl`**：看门狗即可"电源循环"端口，软件自动恢复，不用手拔。
3. 若坚持原生 USB：
   - 固件**不要 reconfig USB 引脚**、**USB 连接时不要进 light sleep**（C3 用 `CONFIG_USJ_NO_AUTO_LS_ON_CONNECTION`；保留 `CONFIG_RTC_CLOCK_BBPLL_POWER_ON_WITH_USB`）；
   - 尽早把 DP/DM mux 切到目标外设（OTG 或 JTAG），避免主机锁在 JTAG；
   - 避免运行中软件复位（看门狗复位）导致 D+ 不释放。
4. **硬件**：C3/S3 用**独立稳定 5V**（不要和电机/舵机共轨），加 470–1000µF 储能 + ESD/CC 电阻；上电顺序 **KickPi → C3 → 电机电源**。
5. **看门狗边界**：总线上完全没有设备时，`usbreset` 无目标；只能记日志/快照（本脚本已加"absent 每 60s 一次 + USB 树/dmesg 快照"）。要自动恢复，必须配合**可控 USB 电源开关/hub**。
