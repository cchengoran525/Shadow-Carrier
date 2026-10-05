#!/bin/sh
# macOS/Linux 一键自动测试(与 run_autotest.bat 等价): 跑临时世界副本, 原始世界永不被改
cd "$(dirname "$0")" || exit 1
export SIM_AUTOTEST=1
SRC=worlds/shadow_carrier.wbt
WORLD=worlds/_autotest.wbt
WEBOTS="${WEBOTS:-/Applications/Webots.app/Contents/MacOS/webots}"
if [ ! -x "$WEBOTS" ]; then
  echo "[autotest] Webots not found at $WEBOTS (set WEBOTS=/path/to/webots)" >&2
  exit 1
fi

cp "$SRC" "$WORLD"
echo "[autotest] world  = $WORLD (temp copy)"
echo "[autotest] webots = $WEBOTS"
echo "[autotest] runs ~110s by itself (keyboard ignored); log: controllers/shadow_carrier/sim_session.jsonl"
"$WEBOTS" --mode=realtime --stdout --stderr "$WORLD"
RC=$?
rm -f "$WORLD"
echo "[autotest] exit code $RC"
exit $RC
