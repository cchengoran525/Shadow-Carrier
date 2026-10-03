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
