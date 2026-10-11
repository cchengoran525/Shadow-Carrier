#!/bin/sh
# 鲁棒性套件: N 个场景 × R 次重复, realtime 逐个跑, 日志归档到 logs/, 最后出报告
# 用法: ./run_robustness.sh [场景列表] [重复次数]
cd "$(dirname "$0")" || exit 1
SCENARIOS="${1:-baseline cross2 occlude bench_block hide_occupied handoff_cross}"
RUNS="${2:-1}"
WEBOTS="${WEBOTS:-/Applications/Webots.app/Contents/MacOS/webots}"
if [ ! -x "$WEBOTS" ]; then
  echo "[rob] Webots not found at $WEBOTS" >&2
  exit 1
fi
mkdir -p logs
for run in $(seq 1 "$RUNS"); do
  for sc in $SCENARIOS; do
    echo "[rob] === scenario=$sc run=$run ==="
    cp worlds/shadow_carrier.wbt worlds/_autotest.wbt
    SIM_AUTOTEST=1 SIM_AUTOTEST_MOTION=1 SIM_SCENARIO="$sc" \
      "$WEBOTS" --mode=realtime --batch --minimize --stdout --stderr worlds/_autotest.wbt \
      > "logs/rob_${sc}_run${run}.stdout" 2>&1
    echo "[rob] rc=$? → logs/rob_${sc}_run${run}.jsonl"
    cp controllers/shadow_carrier/sim_session.jsonl "logs/rob_${sc}_run${run}.jsonl"
  done
done
rm -f worlds/_autotest.wbt
echo "[rob] === 报告 ==="
python3 analyze_robustness.py logs/rob_*.jsonl
