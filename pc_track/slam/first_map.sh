#!/bin/bash
# first_map.sh —— X3Pro 首次建图一条龙：建图节点 + rosbag 录制 + 存图存 posegraph
#
# 前置（缺一不可，脚本会自检）：
#   1. X3Pro 已插 USB（驱动输出 /scan；本机驱动启动方式见仓库外 /tmp/start_x3pro.sh
#      或自己的 x3pro_ws 启动命令）
#   2. source /opt/ros/jazzy/setup.bash
#
# 用法:
#   ./first_map.sh [秒数]        # 默认 60；手持雷达慢速绕房间一圈
#   RVIZ=1 ./first_map.sh 90     # 同时开 rviz2 看建图过程
#
# 产物（pc_track/data/<时间戳>/）:
#   bag_<ts>/        rosbag2（/scan /tf /tf_static，可离线回放复现）
#   map_<ts>.pgm/.yaml   占据栅格地图
#   posegraph_<ts>.posegraph/.data   slam_toolbox 位姿图（可续图/重定位）
set -u
DUR=${1:-60}
HERE="$(cd "$(dirname "$0")" && pwd)"
DATA="$HERE/../data/$(date +%m%d_%H%M%S)"
mkdir -p "$DATA"
# ROS setup.bash 与 set -u 不兼容(AMENT_TRACE_SETUP_FILES 等未绑定变量, 2026-10-24 实测)
set +u
source /opt/ros/jazzy/setup.bash
set -u

# ---- 自检：雷达与驱动 ----
if ! timeout 8 ros2 topic list 2>/dev/null | grep -q '^/scan$'; then
  echo "✗ /scan 不存在：雷达没插，或驱动节点没启动。插上雷达后重试。"
  exit 1
fi
# 数据流自检（2026-10-24 教训：发布者存在 ≠ 数据在流。驱动在 USB 拔出后会静默停止
# 发布且无日志，只查 publisher 会白跑整个窗口）。5 秒内 <10 帧即中止。
FRAMES=$(python3 - <<'EOF' 2>/dev/null
import rclpy, time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
rclpy.init()
n = Node('first_map_preflight')
cnt = [0]
n.create_subscription(LaserScan, '/scan', lambda m: cnt.__setitem__(0, cnt[0]+1), 5)
t0 = time.time()
while rclpy.ok() and time.time()-t0 < 5:
    rclpy.spin_once(n, timeout_sec=0.2)
print(cnt[0])
n.destroy_node(); rclpy.shutdown()
EOF
)
if [ "${FRAMES:-0}" -lt 10 ]; then
  echo "✗ /scan 有发布者但 5 秒只收到 ${FRAMES:-0} 帧——雷达 USB 是否插好？(驱动拔线后会静默停发)"
  exit 1
fi
echo "✓ /scan 数据流正常（5 秒 ${FRAMES} 帧）"

# ---- 1) 恒等静态 TF odom→base_link（手持无里程计必需——2026-10-24 实测：
# 没有它 slam_toolbox 消息过滤器队列塞满丢帧，一帧都进不了图）+ 建图节点（后台）----
ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link \
  >"$DATA/static_tf.log" 2>&1 &
TF_PID=$!
sleep 1
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:="$HERE/mapper_params_x3pro.yaml" >"$DATA/slam.log" 2>&1 &
SLAM_PID=$!
sleep 3
if ! kill -0 $SLAM_PID 2>/dev/null; then
  echo "✗ slam_toolbox 启动失败："; tail -5 "$DATA/slam.log"; exit 1
fi

# ---- 2) 录包（后台）----
ros2 bag record -o "$DATA/bag" /scan /tf /tf_static >"$DATA/record.log" 2>&1 &
REC_PID=$!

# ---- 可选 rviz ----
if [ "${RVIZ:-0}" = "1" ]; then
  rviz2 >"$DATA/rviz.log" 2>&1 &
  RVIZ_PID=$!
fi

echo ""
echo "================ 现在开始走 ================"
echo " 手持雷达，沿房间边缘走一整圈再回起点。三条纪律（E1 实测教训）："
echo "  1. 慢——半速走路(≤0.3 m/s)，比感觉的还要慢一档"
echo "  2. 稳——双手端平，别倾斜/别颠簸（扫描平面倾了墙就'变形'，匹配跟丢）"
echo "  3. 直——多走直线，转身时分几个小步慢慢转"
echo " 剩 ${DUR}s 自动收尾…"
echo "==========================================="
sleep "$DUR"

# ---- 3) 收尾：停录 → 存图 → 存 posegraph → 停建图 ----
kill -INT $REC_PID 2>/dev/null; sleep 2
python3 "$HERE/save_map.py" "$DATA/map" || echo "(!) 存图失败，建图日志见 $DATA/slam.log"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: $DATA/posegraph}" >/dev/null 2>&1 \
  && echo "✓ posegraph 已存" || echo "(!) posegraph 序列化失败"
kill -INT $SLAM_PID 2>/dev/null
kill $TF_PID 2>/dev/null
[ -n "${RVIZ_PID:-}" ] && kill $RVIZ_PID 2>/dev/null
sleep 1

echo ""
echo "==== 产物清单 ($DATA) ===="
ls -lh "$DATA" | awk 'NR>1{printf "  %-40s %s\n", $9, $5}'
echo "回放复现: bash $(dirname $0)/restart_offline_stack.sh $DATA/bag"
