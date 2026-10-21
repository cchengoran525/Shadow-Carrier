#!/bin/bash
# C3 USB watchdog: 侦测 ESP32-C3 (303a:1001) 异常并尽量软件恢复(C3 老毛病: 掉线要插拔).
#
# 恢复阶梯 (从轻到重):
#   1. 设备在 USB、但软链/服务异常 -> usbreset
#   2. 设备完全不在总线(但仍供电) -> xHCI 控制器深度复位 (unbind/bind)
#   3. 深度复位仍无效(可能真掉电) -> 可控 VBUS 电源循环 (GPIO / uhubctl / 自定义命令)
#
# VBUS 后端(三选一, 都不配则跳过):
#   C3_VBUS_GPIO=<gpio号>  C3_VBUS_ACTIVE=1|0   # sysfs GPIO 控 MOSFET/load switch
#   C3_HUB_LOC=<bus> C3_HUB_PORT=<port>          # uhubctl -l .. -p .. -a cycle
#   C3_VBUS_CMD='<shell命令>'                    # 自定义(命令自己完成断电/上电)
# 可选: C3_VBUS_OFF_S=2 (断电秒数), C3_VBUS_SETTLE_S=3 (上电后等待)
#
# 自检: c3_watchdog.sh --selftest-vbus   # 只跑一次 VBUS 钩子(不进入主循环)
LOG=/var/log/c3_watchdog.log
INTERVAL=3
MISS_THRESHOLD=2
ABSENT_LOG_EVERY=60
C3_XHCI="${C3_XHCI:-xhci-hcd.4.auto}"   # 本板 C3 在 usbdrd/fcc00000.dwc3 (bus5)
DEEP_RESET_AFTER="${DEEP_RESET_AFTER:-30}"
DEEP_RESET_COOLDOWN="${DEEP_RESET_COOLDOWN:-60}"
DEEP_RESET="${DEEP_RESET:-1}"

log() { echo "$(date '+%F %T') $*" >> "$LOG"; }

find_c3() {
  for d in /sys/bus/usb/devices/*/; do
    [ -f "$d/idVendor" ] || continue
    if [ "$(cat "$d/idVendor" 2>/dev/null)" = "303a" ] && \
       [ "$(cat "$d/idProduct" 2>/dev/null)" = "1001" ]; then
      basename "$d"; return 0
    fi
  done
  return 1
}

reset_c3() {
  local name="$1" bus dev
  bus=$(printf "%03d" "$(cat "/sys/bus/usb/devices/$name/busnum" 2>/dev/null)")
  dev=$(printf "%03d" "$(cat "/sys/bus/usb/devices/$name/devnum" 2>/dev/null)")
  log "reset via usbreset $bus/$dev ($name)"
  usbreset "$bus/$dev" >>"$LOG" 2>&1
}

deep_reset() {
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

power_cycle() {
  local off="${C3_VBUS_OFF_S:-2}" settle="${C3_VBUS_SETTLE_S:-3}"
  # 1) 自定义命令
  if [ -n "${C3_VBUS_CMD:-}" ]; then
    log "vbus cycle via C3_VBUS_CMD"
    sh -c "$C3_VBUS_CMD" >>"$LOG" 2>&1
    sleep "$settle"
    return 0
  fi
  # 2) uhubctl
  if [ -n "${C3_HUB_LOC:-}" ] && [ -n "${C3_HUB_PORT:-}" ]; then
    if command -v uhubctl >/dev/null 2>&1; then
      log "vbus cycle via uhubctl -l $C3_HUB_LOC -p $C3_HUB_PORT"
      uhubctl -l "$C3_HUB_LOC" -p "$C3_HUB_PORT" -a cycle -d "$off" >>"$LOG" 2>&1
      sleep "$settle"
      return 0
    fi
    log "uhubctl configured but not installed; skip"
  fi
  # 3) sysfs GPIO
  if [ -n "${C3_VBUS_GPIO:-}" ]; then
    local g="/sys/class/gpio/gpio${C3_VBUS_GPIO}"
    local active="${C3_VBUS_ACTIVE:-1}"
    local on="$active" off_gpio=$((1 - active))
    if [ ! -d "$g" ]; then
      echo "${C3_VBUS_GPIO}" > /sys/class/gpio/export 2>>"$LOG"
      sleep 0.2
    fi
    if [ -d "$g" ]; then
      echo out > "$g/direction" 2>>"$LOG"
      log "vbus cycle via GPIO ${C3_VBUS_GPIO} (active=${active}, off=${off_gpio})"
      echo "$off_gpio" > "$g/value" 2>>"$LOG"
      sleep "$off"
      echo "$on" > "$g/value" 2>>"$LOG"
      sleep "$settle"
      return 0
    fi
    log "GPIO ${C3_VBUS_GPIO} export failed; skip"
  fi
  return 1
}

snapshot() {
  {
    echo "--- snapshot $(date '+%F %T') ---"
    lsusb -t 2>&1
    dmesg 2>/dev/null | tail -8
  } >> "$LOG"
}

vbus_configured() {
  [ -n "${C3_VBUS_CMD:-}" ] || \
  { [ -n "${C3_HUB_LOC:-}" ] && [ -n "${C3_HUB_PORT:-}" ]; } || \
  [ -n "${C3_VBUS_GPIO:-}" ]
}

# 自检模式: 只跑一次 VBUS 钩子
if [ "${1:-}" = "--selftest-vbus" ]; then
  log "selftest: vbus configured=$(vbus_configured && echo yes || echo no)"
  if power_cycle; then log "selftest: vbus hook ran"; exit 0; fi
  log "selftest: vbus hook not configured"; exit 1
fi

log "watchdog started (xHCI=$C3_XHCI, deep_reset=$DEEP_RESET, vbus=$(vbus_configured && echo yes || echo no))"
miss=0
present=0
absent_since=0
last_absent_log=0
last_recover=0
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
        if [ $((now - last_absent_log)) -ge "$ABSENT_LOG_EVERY" ]; then
          log "C3 absent from USB (likely power loss / not enumerated) - waiting"
          snapshot
          last_absent_log=$now
        fi
        if [ $((now - absent_since)) -ge "$DEEP_RESET_AFTER" ] && \
           [ $((now - last_recover)) -ge "$DEEP_RESET_COOLDOWN" ]; then
          recovered=0
          [ "$DEEP_RESET" = "1" ] && deep_reset && recovered=1
          if [ "$recovered" -eq 0 ]; then
            if power_cycle; then
              if find_c3 >/dev/null && [ -e /dev/c3_controller ]; then
                log "recovered via VBUS power cycle"
              else
                log "still absent after VBUS power cycle"
              fi
            fi
          fi
          last_recover=$now
        fi
      fi
      miss=0
    fi
  fi
  sleep "$INTERVAL"
done
