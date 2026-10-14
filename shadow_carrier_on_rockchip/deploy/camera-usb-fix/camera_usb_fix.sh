#!/bin/bash
# camera_usb_fix.sh - 保持 ShadowCarrier 相机 USB 常供电(禁用 autosuspend)
#
# 背景: 相机(Sonix 0c45:6366)默认 power/control=auto, suspend/resume 时会
#   re-enumerate(dmesg: usb 7-1 disconnect/reconnect, /dev/videoN 漂移),
#   旧取流进程持有已删节点 → 检测管线死。udev ATTR 写在设备 add 早期不可靠,
#   故用本常驻轮询兜底: 相机一出现即把 power/control 写回 on(<2s 生效)。
# 与 video_stream_v7.py v7.3 的 camera_watchdog 互补(那是相机掉线后的自愈)。
VID=0c45
PID=6366
log() { logger -t camera-usb-fix "$*" 2>/dev/null || true; }

while true; do
  for d in /sys/bus/usb/devices/*; do
    [ -r "$d/idVendor" ] || continue
    [ "$(cat "$d/idVendor" 2>/dev/null)" = "$VID" ] || continue
    [ "$(cat "$d/idProduct" 2>/dev/null)" = "$PID" ] || continue
    ctl="$d/power/control"
    [ -w "$ctl" ] || continue
    cur=$(cat "$ctl" 2>/dev/null)
    if [ "$cur" != "on" ]; then
      if echo on > "$ctl" 2>/dev/null; then
        log "set $d power/control=on (was $cur)"
      fi
    fi
  done
  sleep 2
done
