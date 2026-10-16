#!/bin/sh
# 鲁棒性套件: N 个场景 × R 次重复, realtime 逐个跑, 日志归档到 logs/, 最后出报告
# 用法: ./run_robustness.sh [场景列表] [重复次数]
# 注: passby(擦身掉头) 需要 SIM_TURN=1 + 关掉 ground-truth 掉头假动作, 见下;
#     单独跑 A/B 对照用 ./run_passby.sh
cd "$(dirname "$0")" || exit 1
SCENARIOS="${1:-baseline cross2 occlude bench_block hide_occupied handoff_cross passby}"
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
    EXTRA=""
    if [ "$sc" = "passby" ]; then EXTRA="SIM_TURN=1 SIM_GT_TURN=0"; fi
    cp worlds/shadow_carrier.wbt worlds/_autotest.wbt
    # SIM_LOG 直写归档路径: 并发跑多份仿真时不会互相覆盖(共享 sim_session.jsonl 的老坑)
    SIM_AUTOTEST=1 SIM_AUTOTEST_MOTION=1 SIM_SCENARIO="$sc" $EXTRA \
      SIM_LOG="$PWD/logs/rob_${sc}_run${run}.jsonl" \
      "$WEBOTS" --mode=realtime --batch --minimize --stdout --stderr worlds/_autotest.wbt \
      > "logs/rob_${sc}_run${run}.stdout" 2>&1
    echo "[rob] rc=$? → logs/rob_${sc}_run${run}.jsonl"
  done
done
rm -f worlds/_autotest.wbt
echo "[rob] === 报告 ==="
python3 analyze_robustness.py logs/rob_*.jsonl
