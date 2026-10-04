#!/usr/bin/env python3
"""
polar_mapper.py —— 图像像素 → 机器人极坐标(方位角/地面距离)
输入: 像素坐标 + 云台俯仰/水平指令
输出: (bearing_deg, dist_m)  bearing: 0=车头正前, 右正左负

方法: 射线-地面精确求交
  像素 -> 相机系射线方向 (1, a, b), a=(u-CX)/FX, b=(v-CY)/FY
  相机系 -> 车体系(绕右轴转 tilt_down), 射线与地面 z=0 求交
"""
import math

# ---- 云台线冻结内参 (state/calib/params.py, 勿改) ----
FX = 508.20
FY = 492.70
CX = 292.70
CY = 276.60

# ---- 几何常数 ----
TILT_LEVEL_CMD = 112.0      # 该指令时光轴物理水平
PAN_FORWARD_CMD = 90.58     # 光轴正前方的水平指令角
DEG_PER_CMD = 1.0           # 实测: cmd 90 时上仰约22° => 约1°/指令
H_CAM = 0.10                # 相机离地高度(m), [云台]线实测约10cm, 待精测

# ---- 网格定义 (与[云台]tracker对齐: 8扇区x3距离环) ----
N_SECTORS = 8
SECTOR_DEG = 45.0           # 扇区0=车头正前±22.5°
RINGS_M = [0.7, 1.8, 3.0]   # 近/中/远环外边界(m)
PHI_MIN_DEG = 4.0           # 射线俯角低于此值视为不落地(距离发散)


def tilt_down_deg(tilt_cmd):
    """云台俯仰指令 -> 光轴下俯角(度), 负=上仰"""
    return (tilt_cmd - TILT_LEVEL_CMD) * DEG_PER_CMD


def pixel_to_polar(u, v, tilt_cmd, pan_cmd=None, h_cam=None, tilt_deg=None):
    """像素 -> (bearing_deg, dist_m|None). dist None = 射线不落地/超远
    tilt_deg: 显式俯角(度, 正=下俯)覆盖; None 时用 tilt_cmd 按内建常数推算"""
    if h_cam is None:
        h_cam = H_CAM
    if tilt_deg is None:
        tilt_deg = tilt_down_deg(tilt_cmd)
    t = math.radians(tilt_deg)
    c, s = math.cos(t), math.sin(t)
    a = (u - CX) / FX
    b = (v - CY) / FY
    # 相机系方向(1,a,b) -> 车体系: x前 y右 z上
    dx = c - s * b
    dy = a
    dz = -s - c * b
    if dz >= -math.sin(math.radians(PHI_MIN_DEG)):
        return None, None          # 射线不够俯, 交不到可信地面
    tt = h_cam / (-dz)             # 相机在高度 h_cam
    x = tt * dx
    y = tt * dy
    dist = math.hypot(x, y)
    bearing = math.degrees(math.atan2(y, x))
    if pan_cmd is not None:
        bearing += (pan_cmd - PAN_FORWARD_CMD) * DEG_PER_CMD
    return bearing, dist


def polar_to_sector(bearing_deg, dist_m):
    """极坐标 -> (扇区idx 0..7, 距离环idx 0..3, 3=超远)"""
    b = (bearing_deg + SECTOR_DEG / 2) % 360
    sector = int(b // SECTOR_DEG) % N_SECTORS
    ring = len(RINGS_M)
    for i, rmax in enumerate(RINGS_M):
        if dist_m <= rmax:
            ring = i
            break
    return sector, ring


def ring_range(ring_idx):
    lo = 0.0 if ring_idx == 0 else RINGS_M[ring_idx - 1]
    hi = RINGS_M[ring_idx] if ring_idx < len(RINGS_M) else float("inf")
    return lo, hi
