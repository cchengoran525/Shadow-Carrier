#!/usr/bin/env bash
# V2 麦轮 S3 骨架离线单测：编译纯逻辑层（无 Arduino 依赖）并运行。
# 用法： bash DistributedRobot_S3_MecanumChassis/tests/run_tests.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$(dirname "$SCRIPT_DIR")"
OUT="$SCRIPT_DIR/test_mecanum_bin"

CXX_BIN="${CXX:-}"
if [ -z "$CXX_BIN" ]; then
  for c in g++ c++ clang++; do
    command -v "$c" >/dev/null 2>&1 && { CXX_BIN="$c"; break; }
  done
fi
if [ -z "$CXX_BIN" ]; then
  echo "error: no C++ compiler found (g++/c++/clang++)" >&2
  exit 2
fi

echo "== build: $CXX_BIN =="
"$CXX_BIN" -std=c++17 -Wall -Wextra -Werror -O2 \
  -I"$SRC_DIR" \
  "$SRC_DIR/MecanumKinematics.cpp" \
  "$SRC_DIR/WheelPid.cpp" \
  "$SCRIPT_DIR/test_mecanum.cpp" \
  -o "$OUT"

echo "== run =="
"$OUT"
status=$?

rm -f "$OUT"
exit $status
