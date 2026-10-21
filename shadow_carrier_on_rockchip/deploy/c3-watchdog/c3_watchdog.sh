#!/bin/bash
# C3 USB watchdog: 侦测 ESP32-C3 (303a:1001) 异常并尽量软件恢复(C3 老毛病: 掉线要插拔).
#  - 设备仍在 USB 但软链/服务异常 -> usbreset 复位端口(免插拔)
#  - 设备从 USB 完全消失(多为掉电/未枚举) -> 节流记日志 + 抓快照等待自行枚举
#    (软件救不了"总线上没有的设备"; 见 deploy/c3-watchdog/README.md 的 S3 建议)
LOG=/var/log/c3_watchdog.log
INTERVAL=3
MISS_THRESHOLD=2
ABSENT_LOG_EVERY=60      # absent 时每 60s 记一次(避免 6s 刷屏)

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

snapshot() {  # absent 快照: 记录当前 USB 树 + dmesg 尾部, 便于事后定位
  {
    echo "--- snapshot $(date '+%F %T') ---"
    lsusb -t 2>&1
    dmesg 2>/dev/null | tail -8
  } >> "$LOG"
}

log "watchdog started"
miss=0
present=0
last_absent_log=0
while :; do
  name=$(find_c3) || name=""
  now=$(date +%s)
  if [ -n "$name" ] && [ -e /dev/c3_controller ]; then
    if [ "$present" -eq 0 ]; then
      log "C3 present ($name) - re-enumerated"
      present=1
    fi
    miss=0
  else
    if [ "$present" -eq 1 ]; then
      log "C3 lost (name=${name:-none})"
      present=0
    fi
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
        if [ $((now - last_absent_log)) -ge "$ABSENT_LOG_EVERY" ]; then
          log "C3 absent from USB (likely power loss / not enumerated) - waiting"
          snapshot
          last_absent_log=$now
        fi
      fi
      miss=0
    fi
  fi
  sleep "$INTERVAL"
done
