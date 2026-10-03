#!/usr/bin/env bash
# 固件共享源码同步守卫 (Firmware shared-source sync guard)
#
# 背景: Arduino 要求每个 sketch 目录自带全部源文件, 所以 C3 固件必然有两份拷贝。
# 教训 (2026-08-27): 分叉曾导致功能降级(DIFF 丢失)。此脚本强制 wifi 变体与
# 权威目录 C3_USB_Controller/ 的共享 .h/.cpp 逐字节一致; .ino 允许不同(传输层)。
#
# 用法:
#   tools/check_firmware_sync.sh          # 检查, 不一致则退出码 1
#   tools/check_firmware_sync.sh --fix    # 从权威目录覆盖到 wifi 变体
set -u

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
AUTH="$ROOT/shadow_carrier_on_rockchip/C3_USB_Controller"
VARIANT="$ROOT/wifi/C3_WiFi_Controller"
FIX=0
[ "${1:-}" = "--fix" ] && FIX=1

[ -d "$AUTH" ] || { echo "找不到权威目录: $AUTH"; exit 2; }
[ -d "$VARIANT" ] || { echo "找不到变体目录: $VARIANT"; exit 2; }

fail=0
count=0
for f in "$AUTH"/*.h "$AUTH"/*.cpp; do
  [ -e "$f" ] || continue
  count=$((count + 1))
  b="$(basename "$f")"
  if [ ! -e "$VARIANT/$b" ]; then
    echo "MISSING  $b  (变体目录缺失)"
    [ "$FIX" -eq 1 ] && cp "$f" "$VARIANT/$b"
    fail=1
  elif ! diff -q --strip-trailing-cr "$f" "$VARIANT/$b" >/dev/null 2>&1; then
    echo "DIFF     $b"
    [ "$FIX" -eq 1 ] && cp "$f" "$VARIANT/$b"
    fail=1
  fi
done

if [ "$FIX" -eq 1 ]; then
  echo "已从 $AUTH 同步到 $VARIANT"
  exit 0
fi
if [ "$fail" -eq 0 ]; then
  echo "OK: 共享固件源码一致 ($count 个文件, .ino 传输层允许不同)"
else
  echo
  echo "共享固件源码已分叉! 运行 'tools/check_firmware_sync.sh --fix' 同步, 或手工核对。"
fi
exit $fail
