#!/usr/bin/env bash
# deploy.sh - 从仓库同步规范源码到 KickPi 板 (防目录漂移)
# 用法: ./deploy.sh [user@host] [板端根目录]
# 默认: kickpi@192.168.137.190  ~/shadow_carrier_on_rockchip
# 说明: 单向 仓库→板子; 板上独有文件不受影响; EXCHANGE.md 不在同步范围(本地协调文件)
set -e
KEY="${KEY:-$HOME/.ssh/kickpi_key}"
HOST="${1:-kickpi@192.168.137.190}"
REMOTE_ROOT="${2:-shadow_carrier_on_rockchip}"
LOCAL_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

OPT=(-i "$KEY" -o IdentitiesOnly=yes -o StrictHostKeyChecking=no)

echo "== 同步 $LOCAL_ROOT → $HOST:$REMOTE_ROOT =="
ssh "${OPT[@]}" "$HOST" "mkdir -p ~/$REMOTE_ROOT/{scripts,gimbal,communication,state/calib,C3_USB_Controller}"

scp "${OPT[@]}" -r \
    "$LOCAL_ROOT/scripts/."            "$HOST:$REMOTE_ROOT/scripts/" 
scp "${OPT[@]}" -r \
    "$LOCAL_ROOT/gimbal/."             "$HOST:$REMOTE_ROOT/gimbal/"
scp "${OPT[@]}" -r \
    "$LOCAL_ROOT/communication/."      "$HOST:$REMOTE_ROOT/communication/"
scp "${OPT[@]}" -r \
    "$LOCAL_ROOT/state/."              "$HOST:$REMOTE_ROOT/state/"
scp "${OPT[@]}" -r \
    "$LOCAL_ROOT/C3_USB_Controller/."  "$HOST:$REMOTE_ROOT/C3_USB_Controller/"

echo "== 重启服务 =="
ssh "${OPT[@]}" "$HOST" "echo kickpi | sudo -S systemctl restart rk-control.service video-stream.service 2>/dev/null; systemctl is-active rk-control video-stream"

echo "== 完成。提示: C3 固件改动需另行走 arduino-cli 烧录, 不在本脚本范围 =="
