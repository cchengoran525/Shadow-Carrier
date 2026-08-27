#!/usr/bin/env python3
"""state/decoder.py - 像素 → 车体系方位角解码 (M1)

只做水平方位角(bearing), 不依赖 tilt/相机高度。
约定: 方位角 0° = 当前锁定航向(标定时用的 PAN_BASE),
      正值 = 目标在世界上的右侧(已处理镜像), 由 MIRROR_X 决定符号约定。
"""
import math

from calib.params import FX, CX, K1, K2

# ---- 标定常量 (换机位/换摄像头时更新) ----
# 实测(2026-08-27): 物理左侧目标 u<cx → 解码为正角。
# 统一约定: 方位角正值 = 目标物理左侧 (图像u减小方向)
MIRROR_X = False    # False: 直接用 atan 符号, 不再翻转
PAN_BASE = 79.0     # 标定时的云台水平指令角 (此时定义方位角0°)


def undistort_u(u_norm: float) -> float:
    """归一化u坐标去畸变 (k1,k2 径向, 迭代求解)。u_norm=(u-cx)/fx"""
    ud = u_norm
    for _ in range(8):
        r2 = ud * ud
        ud = u_norm / (1 + K1 * r2 + K2 * r2 * r2)
    return ud


def pixel_to_bearing_offset(u_px: float) -> float:
    """单个像素u -> 相对光轴的物理方位角偏移(度), 已含镜像与畸变。
    实测约定: 正值 = 目标物理左侧 (与拖拽面板方向相反属正常, 摄像头镜像)"""
    u_n = (u_px - CX) / FX
    u_c = undistort_u(u_n)
    ang = math.degrees(math.atan(u_c))
    return ang if not MIRROR_X else -ang


def decode_bearings(detections: list) -> list:
    """输入 yolo_daemon 的 detections 列表, 追加 'bearing_deg' 字段返回。"""
    out = []
    for det in detections:
        d = dict(det)
        cx_box = (d["x1"] + d["x2"]) / 2
        d["bearing_deg"] = round(pixel_to_bearing_offset(cx_box), 2)
        out.append(d)
    return out


if __name__ == "__main__":
    import json
    import urllib.request

    req = urllib.request.Request(
        "http://127.0.0.1:8080/api/detections",
        headers={"User-Agent": "decoder"})
    data = json.loads(urllib.request.urlopen(req, timeout=2).read())
    result = decode_bearings(data.get("detections", []))
    print(json.dumps(result, ensure_ascii=False, indent=1))
