# Camera USB Autosuspend Fix (相机 USB 常供电 / 禁用 autosuspend)

## 背景
- 相机（Sonix `0c45:6366`）默认 `power/control=auto`（`autosuspend_delay_ms=2000`）。
- suspend/resume 时会 **re-enumerate**：`dmesg` 出现 `usb 7-1: USB disconnect` + `new high-speed USB device`，
  `/dev/videoN` 节点漂移。实测约每 ~5 分钟一次。
- 旧取流进程（`cv2.VideoCapture`）持有已删除的 `/dev/videoN` → `cap.read()` 永久阻塞 →
  producer 停发帧 → `yolo_daemon` 静默 → watchdog 空转重启 + `<defunct>` 僵尸 → **检测永久失效**
  （`/api/detections` `age_ms` 120s+、`detections:[]`）。

## 为什么不用纯 udev 规则
`configs/99-shadowcarrier-camera.rules` 里的 `ATTR{power/control}="on"` 在**显式 udevadd 触发**时有效，
但在设备 deauth/re-auth（re-enumerate）路径上抓不到时机 → 仍回落到 `auto`。`RUN+`/`usbcore.autosuspend=-1`
在这些路径同样不可靠。故改用常驻轮询兜底（与 `deploy/c3-watchdog/` 同思路）。

## 能力边界（诚实说明）
| 情况 | 本修复 |
|---|---|
| 相机在 USB 上，`power/control` 被置回 `auto` | ✅ 2s 内写回 `on` |
| 相机 re-enumerate（新节点） | ✅ 新设备出现后 2s 内写回 `on` |
| 相机线材/供电导致**物理掉线** | ❌ 软件无法阻止；由 `video_stream_v7.py` v7.3 的 `camera_watchdog` 掉线自愈（~4s 恢复取流） |
| 硬件级防抖 | 需查线材/供电（相机与 C3 分供电、加电容） |

## 文件
- `camera_usb_fix.sh` —— 每 2s 扫描 USB 设备，凡 `0c45:6366` 且 `power/control≠on` 即写回 `on`（记 syslog）
- `camera-usb-fix.service` —— systemd 常驻（`Restart=always`）

## 安装（板端，需 root）
```bash
sudo install -m 755 camera_usb_fix.sh /usr/local/bin/camera_usb_fix.sh
sudo cp camera-usb-fix.service /etc/systemd/system/camera-usb-fix.service
sudo systemctl daemon-reload
sudo systemctl enable --now camera-usb-fix
```

## 验证
```bash
# 强制模拟一次相机 re-enumerate，观察 2s 内 power/control 被写回 on
sudo sh -c 'echo 0 > /sys/bus/usb/devices/7-1/authorized'; sleep 4
sudo sh -c 'echo 1 > /sys/bus/usb/devices/7-1/authorized'; sleep 4
cat /sys/bus/usb/devices/7-1/power/control          # 期望: on
journalctl -t camera-usb-fix -n 5                   # 期望: set ... power/control=on
bash scripts/health_check.sh                        # 期望: camera usb power/control=on
```
