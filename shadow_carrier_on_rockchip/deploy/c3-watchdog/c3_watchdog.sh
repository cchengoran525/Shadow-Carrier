#!/bin/bash
# C3 USB watchdog: 侦测 ESP32-C3 (303a:1001) 异常并尽量软件恢复(C3 老毛病: 掉线要插拔).
#  - 设备仍在 USB 但软链/服务异常 -> usbreset 复位端口(免插拔)
#  - 设备从 USB 完全消失(多为掉电) -> 记日志等待自行枚举(软件救不了, 需硬件/电源侧)
LOG=/var/log/c3_watchdog.log
INTERVAL=3
MISS_THRESHOLD=2

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

log "watchdog started"
miss=0
while :; do
  name=$(find_c3) || name=""
  if [ -n "$name" ] && [ -e /dev/c3_controller ]; then
    miss=0
  else
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
        log "C3 absent from USB (likely power loss) - waiting for re-enumeration"
      fi
      miss=0
    fi
  fi
  sleep "$INTERVAL"
done
