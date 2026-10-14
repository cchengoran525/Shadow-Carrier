#!/usr/bin/env python3
"""state/decoder.py - 像素 → 车体系方位角解码 (M1)

只做水平方位角(bearing), 不依赖 tilt/相机高度。
输出 = 目标相对【光轴】的方位偏移, 符号随 MIRROR_X:
    MIRROR_X=False (现值) → 偏移 = +atan((u-CX)/FX), 即 u>CX 为正。
⚠️ 跨线契约 (2026-10-14 [世界]符号链审计后定):
    世界方位角约定 = 【右正】(与 world/ polar_mapper / grid.json / 媒体一致)。
    本模块不直接产出世界方位角; 若要, 换算:
        theta_world = (pan - PAN_FORWARD) - decoder_bearing
    权威自检: world/fusion/test_bearing_convention.py (9 断言, 无硬件可跑)。
    ⚠️ 切勿把本偏移与 (pan-PAN_FORWARD) 直接相加 —— 两者是相反的符号约定
    (V1 曾埋此 bug, rk_control 的 owner_bearing 已因此删除)。
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
