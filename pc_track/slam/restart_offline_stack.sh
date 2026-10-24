#!/bin/bash
# restart_offline_stack.sh —— 一键重启离线建图全栈(统一 ROS_LOCALHOST_ONLY=1)
# 用法: bash restart_offline_stack.sh [bag路径]
set -u
BAG=${1:-/tmp/recovered_bag}
export ROS_LOCALHOST_ONLY=1
set +u
source /opt/ros/jazzy/setup.bash
set -u

# 1) 清场: 按 ps 输出杀目标进程(脚本自身 cmdline 不含目标串, 不会自杀)
ps -eo pid,cmd | grep -E 'sync_slam_toolbox_node|static_transform_pub|rosbag2_pla[y]|offline_x3pro|slam_toolbox' \
  | grep -v grep | awk '{print $1}' | xargs -r kill -9 2>/dev/null
sleep 2
rm -f /dev/shm/fastrtps_*

# 2) 静态 TF odom->base_link (手持无里程计的标准补丁)
setsid nohup ros2 run tf2_ros static_transform_publisher \
  --frame-id odom --child-frame-id base_link > /tmp/static_tf.log 2>&1 < /dev/null &
sleep 2

# 3) slam_toolbox (X3Pro 定制配置, sync 离线节点)
cd "$(dirname "$0")"
setsid nohup ros2 launch offline_x3pro_launch.py > /tmp/offline_slam.log 2>&1 < /dev/null &
sleep 7

# 4) 回放 (tf_static 锁存 QoS 覆盖)
setsid nohup ros2 bag play "$BAG" --clock \
  --qos-profile-overrides-path /tmp/tfstatic_qos.yaml > /tmp/bag_play.log 2>&1 < /dev/null &

echo "=== 全栈已启动 (ROS_LOCALHOST_ONLY=1) ==="
grep -c Activating /tmp/offline_slam.log 2>/dev/null | xargs echo "slam Activating 次数:"
pgrep -fc 'rosbag2 pla[y]' | xargs echo "回放进程数:"
