#!/bin/sh
# 仅在 HRI 停驻域(WAIT/HIDE)刷新 grid.json；跟随/移动中不跑（云台会被抢）。
# 退出码恒为 0：没有可跑的条件就安静跳过。
STATE=/dev/shm/hri_state.json
[ -f "$STATE" ] || exit 0
RUN=$(/usr/bin/python3 - "$STATE" <<'PY'
import json, sys, time
try:
    o = json.load(open(sys.argv[1]))
except Exception:
    print(0); raise SystemExit
ok = (time.time() - float(o.get("ts", 0)) < 2.0) \
     and bool(o.get("parked")) \
     and o.get("state") in ("WAIT", "HIDE")
print(1 if ok else 0)
PY
)
[ "$RUN" = "1" ] || exit 0
cd /home/kickpi/shadow_carrier_on_rockchip || exit 0
exec /usr/bin/python3 world/fusion/geometry_daemon.py once
