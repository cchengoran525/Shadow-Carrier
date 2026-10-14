#!/usr/bin/env bash
# health_check.sh - ShadowCarrier 一键健康自检 (demo 前每次开机跑)
# 覆盖: 相机节点/USB常供电 · video-stream 吞吐(fps/age/restarts) · yolo_daemon 存活
#       · dnsmasq CPU/绑定 · 热点(hostapd/AP地址) · IMU · C3 串口 · 系统服务
# 用法:
#   bash health_check.sh                 # 单次快照
#   bash health_check.sh --watch 600 15  # 每 15s 采样一次, 持续 600s (demo 前长跑)
# 退出码: 0 全绿 / 1 有 FAIL / 2 有 WARN
set -u

WATCH=0; INTERVAL=15
if [ "${1:-}" = "--watch" ]; then WATCH="${2:-600}"; INTERVAL="${3:-15}"; fi
API="http://127.0.0.1:8080/api/detections"
SVC="rk-control video-stream ap-hotspot imu c3-watchdog camera-usb-fix"
TH_AGE=200          # ms: 检测结果年龄上限(交付标准 <200)
TH_DNS=10           # %: dnsmasq CPU 上限
STATE=/tmp/health_check.prev

fails=0; warns=0
ok()   { printf "  \033[32m[PASS]\033[0m %s\n" "$*"; }
bad()  { printf "  \033[31m[FAIL]\033[0m %s\n" "$*"; fails=$((fails+1)); }
warn() { printf "  \033[33m[WARN]\033[0m %s\n" "$*"; warns=$((warns+1)); }

snapshot() {
  fails=0; warns=0
  local ts; ts=$(date '+%F %T')
  echo "== ShadowCarrier health @ $ts =="
  # 一次性取 API/dnsmasq/imu
  local api dns cpu imu daemon_rst
  api=$(curl -s --max-time 5 "$API" 2>/dev/null)
  cpu=$(ps -o pcpu= -C dnsmasq 2>/dev/null | head -1 | tr -d ' ')
  imu=$(cat /dev/shm/imu.json 2>/dev/null)

  echo "-- 服务 --"
  for s in $SVC; do
    if systemctl is-active --quiet "$s"; then ok "$s active"; else bad "$s NOT active"; fi
  done

  echo "-- 相机 --"
  if [ -L /dev/camera0 ] && [ -e /dev/camera0 ]; then
    ok "/dev/camera0 -> $(readlink /dev/camera0)"
  else
    bad "/dev/camera0 缺失/悬空 (USB相机未就绪)"
  fi
  local ctl="/sys/bus/usb/devices/7-1/power/control"
  if [ -e "$ctl" ]; then
    [ "$(cat "$ctl")" = "on" ] && ok "camera usb power/control=on (autosuspend 关闭)" \
                                || warn "camera usb power/control=$(cat "$ctl") (应=on; 会周期 re-enumerate)"
  else
    warn "未找到 7-1 电源节点 (相机 bus 可能不同, 不影响取流)"
  fi
  # 最近 USB 相机掉线计数(与上次比较)
  local dc; dc=$(dmesg 2>/dev/null | grep -c '0c45' || true)
  if [ -f "$STATE" ]; then
    local prev; prev=$(grep '^cam_usb_events=' "$STATE" 2>/dev/null | cut -d= -f2)
    [ -n "$prev" ] && { [ "$dc" -gt "$prev" ] && warn "USB相机事件数增加 ($prev -> $dc, 有新枚举)" || ok "USB相机自上次无新增枚举事件 (总 $dc)"; }
  else
    ok "USB相机事件计数基线=$dc"
  fi

  echo "-- 视觉管线 (/api/detections) --"
  if [ -z "$api" ] || ! echo "$api" | grep -q '"fps"'; then
    bad "API 无响应 (video-stream 或 yolo 挂了)"
  else
    read -r fps age inage rst reopen ndet dpid <<EOF
$(echo "$api" | python3 -c '
import sys,json
d=json.load(sys.stdin); s=d.get("stats",{})
print(d.get("fps"), d.get("age_ms"), d.get("input_age_ms"), s.get("restarts"),
      s.get("cam_reopens"), len(d.get("detections") or []), d.get("daemon_pid"))' 2>/dev/null)
EOF
    if [ -z "${age:-}" ] || [ "$age" = "None" ]; then bad "age_ms 缺失(从未收到结果)"; else
      awk -v a="$age" -v t="$TH_AGE" 'BEGIN{exit !(a<t)}' && ok "age_ms=$age (<$TH_AGE) fps=$fps dets=$ndet" \
        || bad "age_ms=$age (>= $TH_AGE; 检测陈旧)"
    fi
    [ -n "${inage:-}" ] && [ "$inage" != "None" ] && ok "input_age_ms=$inage (producer 仍在投喂)" || warn "input_age_ms 缺失(producer 未投喂?)"
    # restarts 是否增长
    if [ -f "$STATE" ]; then
      local prst; prst=$(grep '^restarts=' "$STATE" 2>/dev/null | cut -d= -f2)
      if [ -n "$prst" ]; then
        [ "${rst:-0}" -gt "$prst" ] && warn "yolo restarts 增长 ($prst -> $rst)" || ok "yolo restarts 稳定 ($rst)"
      fi
    else
      ok "yolo restarts=$rst (基线)"
    fi
    [ "${reopen:-0}" -gt 0 ] && warn "cam_reopens=$reopen (相机重连过; 若持续增长查 USB/线材)" || ok "cam_reopens=0"
    # daemon 真实存活
    if [ -n "${dpid:-}" ] && [ "$dpid" != "None" ] && [ -d "/proc/$dpid" ]; then ok "yolo_daemon pid=$dpid 存活"; else bad "yolo_daemon 进程不存活"; fi
  fi
  if pgrep -x yolo_daemon >/dev/null 2>&1; then
    local z; z=$(ps -o stat= -C yolo_daemon 2>/dev/null | grep -c '^Z' || true)
    [ "$z" -gt 0 ] && warn "$z 个 yolo_daemon 僵尸(<defunct>)" || true
  fi

  echo "-- dnsmasq / 热点 --"
  if [ -n "$cpu" ]; then
    awk -v c="$cpu" -v t="$TH_DNS" 'BEGIN{exit !(c<t)}' && ok "dnsmasq CPU=${cpu}% (<$TH_DNS)" || bad "dnsmasq CPU=${cpu}% (跑飞)"
  else
    bad "dnsmasq 未运行"
  fi
  ss -lun 2>/dev/null | grep -qE ':53([[:space:]]|$)' && ok "dnsmasq 监听 UDP:53" || bad "未见 UDP:53 监听 (dnsmasq 未起)"
  pgrep -x hostapd >/dev/null 2>&1 && ok "hostapd 运行 (AP ShadowCarrier-RK)" || bad "hostapd 未运行 (热点挂)"
  ip -4 addr show wlan0 2>/dev/null | grep -q '192.168.4.1' && ok "wlan0=192.168.4.1" || bad "wlan0 无 192.168.4.1"
  if iw dev wlan0 station dump 2>/dev/null | grep -q 'b6:ac:ee:7e:b2:fd'; then
    ok "在场信号: 手机已关联 AP (b6:ac:ee:7e:b2:fd)"
  elif grep -q 'b6:ac:ee:7e:b2:fd' /var/lib/misc/dnsmasq.leases 2>/dev/null; then
    warn "手机未关联, 但 lease 在位 (b6:ac:ee:7e:b2:fd) — 手机可能休眠/不在场"
  else
    warn "未见手机关联/lease (b6:ac:ee:7e:b2:fd)"
  fi

  echo "-- IMU / C3 --"
  if [ -n "$imu" ]; then
    echo "$imu" | python3 -c '
import sys,json,time
d=json.load(sys.stdin); age=time.monotonic()-d.get("ts",0)
print(("  \033[32m[PASS]\033[0m imu.json age=%.2fs yaw_rate=%.2f q=%.1f"%(
    age, d.get("yaw_rate_dps",0), d.get("quality",0))) if age<1.0 else
    ("  \033[33m[WARN]\033[0m imu.json age=%.2fs (陈旧)"%age))' 2>/dev/null || warn "imu.json 解析失败"
  else
    bad "imu.json 缺失"
  fi
  [ -e /dev/ttyACM0 ] && ok "C3 /dev/ttyACM0 在位" || bad "C3 /dev/ttyACM0 缺失"

  # 保存状态供下次比较
  printf 'restarts=%s\ncam_usb_events=%s\n' "${rst:-}" "$dc" > "$STATE"
  echo "== 结果: FAIL=$fails WARN=$warns =="
}

if [ "$WATCH" -gt 0 ]; then
  end_ts=$(( $(date +%s) + WATCH ))
  while [ "$(date +%s)" -lt "$end_ts" ]; do snapshot; echo "---- 下一次 ${INTERVAL}s 后 ----"; sleep "$INTERVAL"; done
  echo "== watch 结束 (FAIL=$fails WARN=$warns) =="
else
  snapshot
fi
[ "$fails" -gt 0 ] && exit 1
[ "$warns" -gt 0 ] && exit 2
exit 0
