#!/bin/sh
# passby A/B 对照: 擦身场景下"底盘转向通道 开/关" (2026-10-16)
#   A (SIM_TURN=1): 原地转/掉头搜索接线 → 期望守住视野
#   B (SIM_TURN=0): 现状失败基线 → 期望擦身后跟丢
# 两跑都关掉 ground-truth 掉头假动作(SIM_GT_TURN=0), 否则验的是舞台效果。
# ⚠️ 每跑用独立 SIM_LOG: 并发跑两份仿真时共享 sim_session.jsonl 会互相截断。
cd "$(dirname "$0")" || exit 1
WEBOTS="${WEBOTS:-/Applications/Webots.app/Contents/MacOS/webots}"
if [ ! -x "$WEBOTS" ]; then
  echo "[passby] Webots not found at $WEBOTS" >&2
  exit 1
fi
mkdir -p logs
for variant in on off; do
  if [ "$variant" = "on" ]; then T=1; else T=0; fi
  echo "[passby] === SIM_TURN=$T (gt_turn=0) ==="
  cp worlds/shadow_carrier.wbt "worlds/_passby_${variant}.wbt"
  SIM_AUTOTEST=1 SIM_AUTOTEST_MOTION=1 SIM_SCENARIO=passby SIM_TURN="$T" SIM_GT_TURN=0 \
    SIM_LOG="$(pwd)/logs/passby_${variant}.jsonl" \
    "$WEBOTS" --mode=realtime --batch --minimize --stdout --stderr \
      "worlds/_passby_${variant}.wbt" \
    > "logs/passby_${variant}.stdout" 2>&1
  echo "[passby] rc=$? → logs/passby_${variant}.jsonl"
  rm -f "worlds/_passby_${variant}.wbt"
done
echo "[passby] === 报告 ==="
python3 analyze_robustness.py logs/passby_on.jsonl logs/passby_off.jsonl
