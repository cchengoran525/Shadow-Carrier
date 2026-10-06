#!/bin/sh
# demo 全场景彩排(运动模式): 跑临时世界副本, 车真的动, ~95s 自动结束
cd "$(dirname "$0")" || exit 1
export SIM_AUTOTEST=1
export SIM_AUTOTEST_MOTION=1
SRC=worlds/shadow_carrier.wbt
WORLD=worlds/_autotest.wbt
WEBOTS="${WEBOTS:-/Applications/Webots.app/Contents/MacOS/webots}"
if [ ! -x "$WEBOTS" ]; then
  echo "[demo] Webots not found at $WEBOTS (set WEBOTS=/path/to/webots)" >&2
  exit 1
fi

cp "$SRC" "$WORLD"
echo "[demo] world  = $WORLD (temp copy, motion ON)"
echo "[demo] webots = $WEBOTS"
echo "[demo] runs ~95s by itself; log: controllers/shadow_carrier/sim_session.jsonl"
"$WEBOTS" --mode=realtime --stdout --stderr "$WORLD"
RC=$?
rm -f "$WORLD"
echo "[demo] exit code $RC"
exit $RC
