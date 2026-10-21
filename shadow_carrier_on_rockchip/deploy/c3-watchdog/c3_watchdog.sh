#!/bin/bash
# C3 USB watchdog: 侦测 ESP32-C3 (303a:1001) 异常并尽量软件恢复(C3 老毛病: 掉线要插拔).
#  - 设备仍在 USB 但软链/服务异常 -> usbreset 复位端口(免插拔)
#  - 设备从 USB 完全消失 -> 先记日志; 持续 ABSENT 后做 **xHCI 控制器深度复位**
#    (unbind/bind C3 所在控制器; 2026-10-21 实测: C3 1s 内重枚举, 相机在另一控制器不受影响)
#  - 深度复位也救不了"真掉电" -> 仍需硬件(独立 5V/可控 VBUS/hub); 见 README
LOG=/var/log/c3_watchdog.log
INTERVAL=3
MISS_THRESHOLD=2
ABSENT_LOG_EVERY=60          # absent 时每 60s 记一次(避免 6s 刷屏)
C3_XHCI="${C3_XHCI:-xhci-hcd.4.auto}"   # 本板 C3 在 usbdrd/fcc00000.dwc3 (bus5)
DEEP_RESET_AFTER="${DEEP_RESET_AFTER:-30}"      # 持续 absent 多少秒后深度复位
DEEP_RESET_COOLDOWN="${DEEP_RESET_COOLDOWN:-60}" # 两次深度复位最小间隔
DEEP_RESET="${DEEP_RESET:-1}"                   # 0=关闭深度复位

log() { echo "$(date '+%F %T') $*" >> "$LOG"; }

find_c3() {   # -> USB 设备名如 5-1
  for d in /sys/bus/usb/devices/*/; do
    [ -f "$d/idVendor" ] || continue
    if [ "$(cat "$d/idVendor" 2>/dev/null)" = "303a" ] && \
       [ "$(cat "$d/idProduct" 2>/dev/null)" = "1001" ]; then
      basename "$d"; return 0
    fi
  done
  return 1
}

reset_c3() {  # usbreset 指定设备 (用法: usbreset BBB/DDD)
  local name="$1"
  local bus dev
  bus=$(printf "%03d" "$(cat "/sys/bus/usb/devices/$name/busnum" 2>/dev/null)")
  dev=$(printf "%03d" "$(cat "/sys/bus/usb/devices/$name/devnum" 2>/dev/null)")
  log "reset via usbreset $bus/$dev ($name)"
  usbreset "$bus/$dev" >>"$LOG" 2>&1
}

deep_reset() {  # C3 完全不在总线时, 复位它所在的 xHCI 控制器(不影响另一控制器的相机)
  [ "$DEEP_RESET" = "1" ] || return 1
  local drv=/sys/bus/platform/drivers/xhci-hcd
  [ -e "$drv/unbind" ] || { log "deep reset skipped: no $drv"; return 1; }
  log "deep reset xHCI $C3_XHCI (unbind/bind)"
  echo "$C3_XHCI" > "$drv/unbind" 2>>"$LOG"
  sleep 2
  echo "$C3_XHCI" > "$drv/bind" 2>>"$LOG"
  sleep 3
  if find_c3 >/dev/null && [ -e /dev/c3_controller ]; then
    log "recovered via xHCI deep reset"
    return 0
  fi
  log "still absent after xHCI deep reset"
  return 1
}

snapshot() {  # absent 快照: 记录当前 USB 树 + dmesg 尾部, 便于事后定位
  {
    echo "--- snapshot $(date '+%F %T') ---"
    lsusb -t 2>&1
    dmesg 2>/dev/null | tail -8
  } >> "$LOG"
}

log "watchdog started (xHCI=$C3_XHCI, deep_reset=$DEEP_RESET)"
miss=0
present=0
absent_since=0
last_absent_log=0
last_deep=0
while :; do
  name=$(find_c3) || name=""
  now=$(date +%s)
  if [ -n "$name" ] && [ -e /dev/c3_controller ]; then
    if [ "$present" -eq 0 ]; then
      log "C3 present ($name) - re-enumerated"
      present=1
    fi
    miss=0
    absent_since=0
  else
    if [ "$present" -eq 1 ]; then
      log "C3 lost (name=${name:-none})"
      present=0
      absent_since=$now
    fi
    [ "$absent_since" -eq 0 ] && absent_since=$now
    miss=$((miss + 1))
    if [ "$miss" -ge "$MISS_THRESHOLD" ]; then
      if [ -n "$name" ]; then
        reset_c3 "$name"
        sleep 4
        if [ -e /dev/c3_controller ]; then
          log "recovered (symlink present again)"
        else
          log "symlink still missing; restarting rk-control"
          systemctl restart rk-control 2>>"$LOG"
        fi
      else
        # 完全不在总线: 节流日志 + 到点做 xHCI 深度复位
        if [ $((now - last_absent_log)) -ge "$ABSENT_LOG_EVERY" ]; then
          log "C3 absent from USB (likely power loss / not enumerated) - waiting"
          snapshot
          last_absent_log=$now
        fi
        if [ $((now - absent_since)) -ge "$DEEP_RESET_AFTER" ] && \
           [ $((now - last_deep)) -ge "$DEEP_RESET_COOLDOWN" ]; then
          deep_reset
          last_deep=$now
        fi
      fi
      miss=0
    fi
  fi
  sleep "$INTERVAL"
done
