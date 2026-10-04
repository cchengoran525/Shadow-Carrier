#!/bin/sh
# imu_start.sh - 启动 IMU 常驻读取器 (后台, 日志到 /tmp/imu_daemon.log)
# 用法: sudo sh imu_start.sh [--cal 秒数]
pkill -f "[i]mu_daemon" 2>/dev/null
sleep 0.5
setsid python3 /home/kickpi/shadow_carrier_on_rockchip/scripts/imu_daemon.py "$@" \
    > /tmp/imu_daemon.log 2>&1 < /dev/null &
sleep 1
echo "IMU daemon 启动: $(pgrep -f '[i]mu_daemon' | head -1)"
