#!/usr/bin/env python3
"""ros_scan_bridge.py —— 扫描生产者·路线 2（ROS 2 /scan → lidar_scan.json 契约）

V2_LIDAR_AVOIDANCE.md §3 的三段解耦里, 生产者有两条可互换路线:
  路线 1(原计划): YDLidar-SDK aarch64 直连, ~60 行桥(待板子, 见附 B-1)
  路线 2(本文件): ROS 2 驱动(ydlidar_ros2_driver) → 本桥 → 同一个契约文件
两路线产出完全相同的帧, guard/daemon/rk_control 零改动。2026-10-24 已在
PC 科研轨(t2linux Ubuntu 24.04 + ROS 2 Jazzy + X3Pro)端到端实测, 见附 D。

数据流:
  /scan (sensor_msgs/LaserScan, 10Hz) ──▶ 本桥 ──▶ /dev/shm/lidar_scan.json

本桥做的三件事(写 SDK 版桥时同样要照抄):
  1. 方位角转换: ROS 左为正(CCW) → 契约右为正(车头 0°, 与
     test_bearing_convention.py 同源) → 取负
  2. 无效点过滤: 驱动对无回波输出 inf / 0.000x 毫米级噪点 → 直接丢弃
     (guard 的 range_min_m 近场滤波是第二道, 这里不送垃圾)
  3. ts 写 time.monotonic(): guard.update() 用单调时钟算 scan_age
     (guard 现已双时钟兼容, 但 monotonic 是约定首选)

用法:
  source /opt/ros/<distro>/setup.bash
  python3 ros_scan_bridge.py                     # 常驻
  LIDAR_SCAN_JSON=/tmp/x.json python3 ros_scan_bridge.py   # 自定义输出
"""
import json
import math
import os
import tempfile
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

OUT = os.environ.get("LIDAR_SCAN_JSON", "/dev/shm/lidar_scan.json")


def write_atomic(path: str, obj) -> None:
    """原子替换写入(契约要求: tmp + rename, 消费方读到的是完整帧)"""
    d = os.path.dirname(path)
    fd, tmp = tempfile.mkstemp(dir=d, suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(obj, f, ensure_ascii=False)
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


class ScanBridge(Node):
    def __init__(self):
        super().__init__("ros_scan_bridge")
        self.frame_id = 0
        self.last_seq = None
        self.create_subscription(LaserScan, "/scan", self.on_scan, 1)

    def on_scan(self, msg: LaserScan) -> None:
        seq = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if seq == self.last_seq:      # 去重: 同一帧被 QoS 重发
            return
        self.last_seq = seq
        pts = []
        a = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and r > 0.0:
                az = -math.degrees(a)             # ROS 左正 → 契约右正
                az = (az + 180.0) % 360.0 - 180.0
                pts.append([round(az, 2), round(r, 3)])
            a += msg.angle_increment
        if not pts:
            return
        self.frame_id += 1
        frame = {"frame_id": self.frame_id, "ts": time.monotonic(), "points": pts}
        write_atomic(OUT, frame)


def main():
    rclpy.init()
    node = ScanBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
