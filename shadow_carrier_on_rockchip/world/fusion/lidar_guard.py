#!/usr/bin/env python3
"""lidar_guard.py —— V2 激光雷达避障核（纯逻辑, 无硬件, 零第三方依赖）

职责边界（详见 docs/V2_LIDAR_AVOIDANCE.md）:
  雷达在 SC 上只有三件事，本模块只做第 ② 件:
    ① HIDE 几何源（门/墙/开阔区）→ 归 geometry_daemon 的 lidar 源, 不在这里
    ② **安全**: 把 360° 扫描 → **运动约束**（限速 / 停止 / 绕行提示）
    ③ SLAM/重定位 → 明确移出机器人范围（PC 科研轨）
  它不做社交决策（谁让谁、要不要分开）——那是 HRI 的事; 它只回答
  "以当前速度朝这个方向走, 会不会撞 / 最多能多快"。

两条契约:
  A. 方位角约定 = 【0°=车头正前, 正=物理右】——与 [世界] polar_mapper / grid.json
     同源; 权威自检 world/fusion/test_bearing_convention.py。
  B. 输入 = 扫描帧 dict {"ts": float, "points": [[bearing_deg, range_m], ...]}
     （也可给 {"angles":[...], "ranges":[...]} 两种标准雷达输出格式）
     输出 = 安全包络 dict —— 落盘即 /dev/shm/lidar_safety.json（同构）。

扫描平面几何（V2 §15.7 修订版决议：雷达藏两层板夹层, z≈13.5cm）:
  - 车体自遮挡只有 4 根铜柱（Ø6mm @ 距传感器 ~17.5cm → 每根 ~2°）→ 角度遮罩
    + 近场滤波（柱子本身就在 range_min 之内, 双保险）;
  - 轮子（顶 8cm）、上层板（17.5cm）、桅杆、篮子 **都不在平面内** → 不需要遮;
  - 单线平面只看得见"穿过 13.5cm 高度"的东西: 腿/椅子腿/箱子/墙 ✓,
    玻璃 ×、<13.5cm 的矮物 ×、高处悬挑（桌面/桌面下沿）×。

用法:
  python3 lidar_guard.py <scan.json>        # 离线跑一帧, 打印安全包络
  import lidar_guard; guard = lidar_guard.Guard(); env = guard.update(frame)
"""
import json
import math
import sys
import time

# ============ 调参区（全部可被 configs/lidar_guard.json 覆盖） ============

# --- 车体几何 (m): V2 占地 30×34cm, 麦轮, 传感器在几何中心附近 ---
BODY_HALF_W = 0.15        # 左右半宽 (x)
BODY_HALF_L = 0.17        # 前后半长 (y)
BODY_RADIUS = math.hypot(BODY_HALF_W, BODY_HALF_L)   # 外接圆半径 0.227 (原地转扫描圈)
SENSOR_X = 0.0            # 传感器相对车体中心 (右正)
SENSOR_Y = 0.0            # (前正)
SENSOR_YAW_DEG = 0.0      # 安装朝向偏差(机器人方位 = 测得方位 + 此值)

# --- 自遮挡遮罩: 由铜柱几何自动生成 (改安装位置只改这两行) ---
PILLARS_XY = [(0.105, 0.14), (-0.105, 0.14), (0.105, -0.14), (-0.105, -0.14)]
PILLAR_R_M = 0.003        # Ø6mm
MASK_MARGIN_DEG = 2.0     # 遮罩余量(安装公差/柱体反光)
MASK_DEG = []             # 手工追加遮罩 [[a0,a1], ...] (机器人方位角, 可跨 0°)

# --- 量程与近场 ---
RANGE_MIN_M = 0.24        # 近场滤波: 比这更近的回波视为自反射(铜柱/板边)
RANGE_MAX_M = 8.0         # X3 Pro 实用距离
SPIKE_KEEP_M = 1.2        # 远处孤立单点(飞点)丢弃阈值; 近于此值一律保留(保守)
SPIKE_DENSE_N = 100       # "稠密帧"点数门: 稀疏帧不做孤立点剔除(可能是真的少回波)
DEDUP_DEG = 0.5           # 同角去重门(单回波雷达一个方位只有一次回波)
MIN_POINTS_OK = 20        # 有效点少于此值 → 判 suspect(探头被挡/雾/脏), 不假装"全清"

# --- 制动模型 (决定"多快算太快") ---
T_REACT_S = 0.35          # 反应时延: 扫描周期 + 环路 + 传输 + 固件缓启
A_BRAKE = 0.6             # 减速度 m/s² (室内舒适上限, 紧急可更大)
GAP_MARGIN_M = 0.08       # 停稳后保留的间隙
V_NOM_MPS = 0.6           # 标称巡航速度(§12.3); 包络只做"限到多少", 不做"提到多少"
V_MIN_MPS = 0.15          # 低于此速度视为"不值得走" → 判 blocked
HARD_GAP_M = 0.05         # 障碍已进入"车体+5cm" → 立即否决, 不等确认窗
CORRIDOR_EXTRA_M = 0.05   # 走廊额外放宽(贴边余量)

# --- 原地转/弧线的转向约束 ---
WZ_MAX_DPS = 90.0         # 允许的最大角速度
WZ_MIN_DPS = 5.0          # 低于此角速度视为"不值得转" → 判 blocked
ROT_GAP_FULL_M = 0.20     # 四周净空 ≥ 此值 → 允许全速转
ROT_GAP_ZERO_M = 0.05     # 净空 ≤ 此值 → 禁止转

# --- 命令单位换算（包络是 SI, 线上协议不是）---
SPD_MAX = 255.0           # MOVE/DIFF 的 PWM 满量程
V_AT_SPD_MAX = 1.0        # 满 PWM 对应的地面速度 m/s(标称; 空载实测后校正)
MEC_V_UNITS = 100.0       # MEC 的 vx/vy 线单位/cm·s⁻¹ 每 m·s⁻¹ (§2.5: 单位 cm/s)
MEC_W_UNITS = 0.0174533   # MEC 的 wz 线单位/rad·s⁻¹ 每 deg·s⁻¹ (§2.5: 单位 rad/s)
# MOVE L/R 语义: V1 固件 = 原地旋转(坦克转向) → 受转向净空约束;
# V2 麦轮若改成真横移, 置 False → 改受横向走廊约束
MOVE_LR_IS_ROTATION = True

# --- 迟滞 (防抖: 单帧噪声不得推动整车) ---
BLOCK_CONFIRM_S = 0.25    # 原始判 blocked 持续多久才认
CLEAR_CONFIRM_S = 0.40    # 恢复需持续多久
RESUME_MARGIN_M = 0.15    # 退出 blocked 需要的额外净空

# --- 健康与降级 ---
STALE_TIMEOUT_S = 0.6     # 扫描帧超过此龄 → stale
DEGRADE_SPEED_MPS = 0.30  # stale 时允许的速度上限(cap 模式)

# --- 簇/行人 ---
CLUSTER_GAP_DEG = 6.0     # 相邻点间隔 > 此值 → 断开成两簇
LEGS_WIDTH_M = (0.05, 0.45)   # 腿级目标宽度带(经典 leg detection)
LEGS_RANGE_M = (0.30, 5.0)
TRACK_ASSOC_DEG = 10.0    # 帧间关联门(角度)
TRACK_ASSOC_M = 0.6       # 帧间关联门(距离, 附加 range*0.2)
TRACK_MAX_MISS = 3
DYNAMIC_MPS = 0.15        # 径向速度超此值 → 判为动目标

DEFAULT_CFG = {}


def _load_defaults():
    """模块常量 → 默认配置字典（供 config 覆盖）"""
    return {
        "body_half_w": BODY_HALF_W, "body_half_l": BODY_HALF_L,
        "sensor_x": SENSOR_X, "sensor_y": SENSOR_Y, "sensor_yaw_deg": SENSOR_YAW_DEG,
        "pillars_xy": PILLARS_XY, "pillar_r_m": PILLAR_R_M,
        "mask_margin_deg": MASK_MARGIN_DEG, "mask_deg": MASK_DEG,
        "range_min_m": RANGE_MIN_M, "range_max_m": RANGE_MAX_M,
        "spike_keep_m": SPIKE_KEEP_M, "spike_dense_n": SPIKE_DENSE_N,
        "dedup_deg": DEDUP_DEG,
        "min_points_ok": MIN_POINTS_OK,
        "t_react_s": T_REACT_S, "a_brake": A_BRAKE, "gap_margin_m": GAP_MARGIN_M,
        "v_nom_mps": V_NOM_MPS, "v_min_mps": V_MIN_MPS,
        "hard_gap_m": HARD_GAP_M, "corridor_extra_m": CORRIDOR_EXTRA_M,
        "wz_max_dps": WZ_MAX_DPS, "wz_min_dps": WZ_MIN_DPS,
        "rot_gap_full_m": ROT_GAP_FULL_M, "rot_gap_zero_m": ROT_GAP_ZERO_M,
        "spd_max": SPD_MAX, "v_at_spd_max": V_AT_SPD_MAX,
        "mec_v_units": MEC_V_UNITS, "mec_w_units": MEC_W_UNITS,
        "move_lr_is_rotation": MOVE_LR_IS_ROTATION,
        "block_confirm_s": BLOCK_CONFIRM_S, "clear_confirm_s": CLEAR_CONFIRM_S,
        "resume_margin_m": RESUME_MARGIN_M,
        "stale_timeout_s": STALE_TIMEOUT_S, "degrade_speed_mps": DEGRADE_SPEED_MPS,
        "cluster_gap_deg": CLUSTER_GAP_DEG,
        "legs_width_m": LEGS_WIDTH_M, "legs_range_m": LEGS_RANGE_M,
        "track_assoc_deg": TRACK_ASSOC_DEG, "track_assoc_m": TRACK_ASSOC_M,
        "dynamic_mps": DYNAMIC_MPS,
    }


def load_config(path=None, overrides=None):
    """默认配置 → 文件覆盖 → 显式覆盖。文件缺失/字段坏掉一律回退默认(不抛)。
    path 传 dict 时按 overrides 处理(防 `load_config({"k": v})` 这个手滑)。"""
    if isinstance(path, dict) and overrides is None:
        path, overrides = None, path
    cfg = _load_defaults()
    if path:
        try:
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
            cfg.update({k: v for k, v in data.items() if not k.startswith("_")})
        except Exception as e:
            print(f"[guard] config {path} 读取失败({e}), 用默认值")
    if overrides:
        cfg.update(overrides)
    # 派生量
    cfg["_body_radius"] = math.hypot(cfg["body_half_w"], cfg["body_half_l"])
    cfg["_masks"] = _build_masks(cfg)
    cfg["_pillar_bearings"] = [m[2] for m in cfg["_masks"]]
    return cfg


# ============================ 遮罩 / 预处理 ============================

def _ang_diff(a, b):
    """两角之差, 归一到 (-180, 180]（度）"""
    return (a - b + 180.0) % 360.0 - 180.0


def _build_masks(cfg):
    """遮罩 = 铜柱几何自动生成 + 手工 mask_deg。返回 [(a0, a1, center_deg), ...]"""
    masks = []
    sx, sy = cfg["sensor_x"], cfg["sensor_y"]
    for (px, py) in cfg["pillars_xy"]:
        d = math.hypot(px - sx, py - sy)
        if d <= 1e-6:
            continue
        center = math.degrees(math.atan2(px - sx, py - sy))   # 右正, 0=前
        half = math.degrees(math.atan2(cfg["pillar_r_m"], d)) + cfg["mask_margin_deg"]
        masks.append(((center - half) % 360.0, (center + half) % 360.0, round(center, 1)))
    for (a0, a1) in cfg.get("mask_deg") or []:
        masks.append((float(a0) % 360.0, float(a1) % 360.0, None))
    return masks


def is_masked(bearing_deg, cfg):
    """机器人方位角是否落在遮罩内（支持跨 0° 的区间）"""
    for (a0, a1, _c) in cfg["_masks"]:
        if a0 <= a1:
            if a0 <= bearing_deg <= a1:
                return True
        elif bearing_deg >= a0 or bearing_deg <= a1:   # 跨 0°
            return True
    return False


def mask_spans_deg(cfg):
    """遮罩区间列表(供落盘/诊断)"""
    return [[round(a0, 1), round(a1, 1)] for (a0, a1, _c) in cfg["_masks"]]


def parse_frame(frame):
    """扫描帧 → (ts, points) ；points 保持原始 (bearing_deg, range_m)。
    支持三种输入: {"points": [[deg, m], ...]} / {"angles": [...], "ranges": [...]}
    / [[deg, m], ...]"""
    if isinstance(frame, list):
        return None, [(float(p[0]), float(p[1])) for p in frame]
    ts = frame.get("ts")
    if "points" in frame and frame["points"]:
        pts = [(float(p[0]), float(p[1])) for p in frame["points"]]
    elif "angles" in frame and "ranges" in frame:
        pts = [(float(a), float(r)) for a, r in zip(frame["angles"], frame["ranges"])]
    else:
        pts = []
    return ts, pts


def to_robot_frame(points, cfg):
    """(原始方位, 距离) → [(bearing_robot_deg, range_m, x_right, y_fwd)]，
    去掉遮罩角/量程外/近场自反射/远处飞点。"""
    out = []
    syaw = cfg["sensor_yaw_deg"]
    for (b, r) in points:
        if r is None or r <= 0:
            continue
        if r < cfg["range_min_m"] or r > cfg["range_max_m"]:
            continue
        rb = _ang_diff(b + syaw, 0.0)
        if is_masked(rb, cfg):
            continue
        out.append((rb, r))
    out.sort(key=lambda p: p[0])
    # 同角去重: 单回波雷达一个方位只有一次回波; 同方位双点(多回波/抖动)取更近的那个
    dedup = []
    for (rb, r) in out:
        if dedup and abs(rb - dedup[-1][0]) < cfg["dedup_deg"]:
            if r < dedup[-1][1]:
                dedup[-1] = (rb, r)
            continue
        dedup.append((rb, r))
    out = dedup
    # 飞点滤波: 远处孤立点(±3° 内无邻点, 或与邻点中位距离差过大) → 丢弃;
    # 近于 spike_keep_m 的一律保留(保守: 近处误停便宜, 漏停昂贵)
    keep = []
    n = len(out)
    for i, (rb, r) in enumerate(out):
        if r <= cfg["spike_keep_m"]:
            keep.append((rb, r))
            continue
        nb = [out[j][1] for j in range(max(0, i - 2), min(n, i + 3)) if j != i
              and abs(_ang_diff(out[j][0], rb)) <= 3.0]
        if not nb:
            # 3° 内无任何邻点: 稠密帧里不存在(飞点) → 丢; 稀疏帧可能真的只有几个回波 → 留
            if n >= cfg["spike_dense_n"]:
                continue
            keep.append((rb, r))
            continue
        if len(nb) >= 2:
            med = sorted(nb)[len(nb) // 2]
            if abs(r - med) >= max(0.30, 0.45 * med):
                continue
        keep.append((rb, r))
    res = []
    for (rb, r) in keep:
        x = cfg["sensor_x"] + r * math.sin(math.radians(rb))
        y = cfg["sensor_y"] + r * math.cos(math.radians(rb))
        res.append((rb, r, x, y))
    return res


# ============================ 扇区 / 聚类 ============================

def sector_minima(points, cfg, n_sectors=24):
    """按方位分扇区取最小距离 → (ranges, counts)；空扇区 range=None
    扇区 0 = 车头正前 ±(360/2n)°，扇区号向【右】递增（与 polar_mapper 一致）"""
    width = 360.0 / n_sectors
    sec = [(None, 0)] * n_sectors
    for (rb, r, _x, _y) in points:
        idx = int((rb + width / 2.0) % 360.0 // width) % n_sectors
        cur_r, cur_n = sec[idx]
        sec[idx] = (r if cur_r is None else min(cur_r, r), cur_n + 1)
    return [s[0] for s in sec], [s[1] for s in sec]


def cluster_points(points, cfg):
    """角向聚类: 相邻点间隔 > cluster_gap_deg → 断开。
    输出簇: {bearing_deg, range_m, width_m, n, cls, x, y, idxs}"""
    if not points:
        return []
    clusters = []
    cur = [0]
    for i in range(1, len(points)):
        if points[i][0] - points[cur[-1]][0] > cfg["cluster_gap_deg"]:
            clusters.append(cur)
            cur = [i]
        else:
            cur.append(i)
    clusters.append(cur)

    out = []
    for idxs in clusters:
        cl = [points[i] for i in idxs]
        rs = [p[1] for p in cl]
        bs = [p[0] for p in cl]
        r_min = min(rs)
        # 最近点定方位; 宽度用角宽×距离近似(小角近似)
        i_near = rs.index(r_min)
        b_near = bs[i_near]
        span = math.radians(max(bs) - min(bs))
        width = 2.0 * r_min * math.sin(span / 2.0) if len(cl) > 1 else 0.02
        lo, hi = cfg["legs_width_m"]
        rlo, rhi = cfg["legs_range_m"]
        is_legs = (len(cl) >= 2 and lo <= width <= hi and rlo <= r_min <= rhi)
        out.append({"bearing_deg": round(b_near, 1), "range_m": round(r_min, 2),
                    "width_m": round(width, 2), "n": len(cl),
                    "cls": "legs" if is_legs else "obstacle",
                    "x": cl[i_near][2], "y": cl[i_near][3], "idxs": idxs})
    return out


# ============================ 净空 / 速度律 ============================

def _rect_support(theta_rad, cfg):
    """矩形车体在行进方向 θ 的支撑函数 → (沿行进半长, 横向半宽)"""
    c, s = abs(math.cos(theta_rad)), abs(math.sin(theta_rad))
    along = cfg["body_half_l"] * c + cfg["body_half_w"] * s
    across = cfg["body_half_l"] * s + cfg["body_half_w"] * c
    return along, across


def clearance_dir(points, cfg, theta_deg):
    """朝 θ(机器人方位, 0=前, 正=右) 行进时的车头间隙:
    只统计落在"行进走廊"(|横向|≤车体横半宽+余量)内的点,
    间隙 = 沿行进方向投影 - 车体沿行进半长。
    返回 (gap_m|None, 最近点idx|None)"""
    th = math.radians(theta_deg)
    along_half, across_half = _rect_support(th, cfg)
    across_lim = across_half + cfg["corridor_extra_m"]
    best, best_range, best_i = None, None, None
    for i, (rb, r, x, y) in enumerate(points):
        along = y * math.cos(th) + x * math.sin(th)
        lat = x * math.cos(th) - y * math.sin(th)
        if abs(lat) > across_lim:
            continue
        gap = along - along_half
        if gap < -0.10:      # 已经被"甩在身后"的点不计
            continue
        # 间隙并列时取更近的点(平面墙的所有点 along 相同 → 报正对的那一点)
        if best is None or (gap, r) < (best, best_range):
            best, best_range, best_i = gap, r, i
    return best, best_i


def rotation_gap(points, cfg):
    """原地/弧线转向的净空: 最近点半径 - 外接圆半径"""
    if not points:
        return None
    rho_min = min(math.hypot(p[2], p[3]) for p in points)
    return rho_min - cfg["_body_radius"]


def v_from_gap(gap_m, cfg):
    """间隙 → 允许速度: v·t_react + v²/(2a) ≤ gap - margin 的解"""
    if gap_m is None:
        return cfg["v_nom_mps"]
    d = gap_m - cfg["gap_margin_m"]
    if d <= 0:
        return 0.0
    v = cfg["a_brake"] * (math.sqrt(cfg["t_react_s"] ** 2 + 2.0 * d / cfg["a_brake"])
                          - cfg["t_react_s"])
    return max(0.0, min(v, cfg["v_nom_mps"]))


def gap_required(v_mps, cfg):
    """速度为 v 时需要的间隙（v_from_gap 的反函数, 供恢复判据用）"""
    return v_mps * cfg["t_react_s"] + v_mps * v_mps / (2.0 * cfg["a_brake"]) + cfg["gap_margin_m"]


def wz_from_gap(rot_gap_m, cfg):
    if rot_gap_m is None:
        return cfg["wz_max_dps"]
    g = rot_gap_m - cfg["rot_gap_zero_m"]
    if g <= 0:
        return 0.0
    frac = min(1.0, g / max(1e-6, cfg["rot_gap_full_m"] - cfg["rot_gap_zero_m"]))
    return round(cfg["wz_max_dps"] * frac, 1)


# ============================ 目标跟踪（动/静判别） ============================

class Tracker:
    """极坐标下的最近邻关联跟踪: 供"动目标提前量"与"腿级人"输出。
    简化到能离线断言的程度——不做全局最优关联(点数少, 贪心够)。"""

    def __init__(self):
        self.tracks = {}      # id -> {bearing, range, rr, miss, cls, n}
        self._next = 1

    def update(self, clusters, dt):
        for t in self.tracks.values():
            t["miss"] += 1
        for cl in clusters:
            best, best_d = None, None
            for tid, t in self.tracks.items():
                dr = abs(cl["range_m"] - t["range"])
                db = abs(_ang_diff(cl["bearing_deg"], t["bearing"]))
                if db > 10.0 or dr > 0.6 + 0.2 * t["range"]:
                    continue
                d = dr + db * 0.02 * t["range"]
                if best_d is None or d < best_d:
                    best, best_d = tid, d
            if best is None:
                tid = self._next
                self._next += 1
                self.tracks[tid] = {"bearing": cl["bearing_deg"], "range": cl["range_m"],
                                    "rr": 0.0, "miss": 0, "cls": cl["cls"], "n": cl["n"]}
            else:
                t = self.tracks[best]
                rr = (cl["range_m"] - t["range"]) / dt if dt > 1e-3 else 0.0
                t["rr"] = 0.5 * t["rr"] + 0.5 * rr          # EMA
                t["bearing"], t["range"] = cl["bearing_deg"], cl["range_m"]
                t["miss"], t["cls"], t["n"] = 0, cl["cls"], cl["n"]
            cl["track_id"] = best if best is not None else (self._next - 1)
            cl["closing_mps"] = round(max(0.0, -self.tracks[cl["track_id"]]["rr"]), 2)
            cl["dynamic"] = abs(self.tracks[cl["track_id"]]["rr"]) > 0.15
        dead = [tid for tid, t in self.tracks.items() if t["miss"] > 3]
        for tid in dead:
            del self.tracks[tid]
        return clusters


# ============================ 安全包络 ============================

class Guard:
    """有状态避障核: update(frame, now) → 安全包络 dict（可直接落盘/消费）"""

    def __init__(self, cfg=None, config_path=None):
        self.cfg = cfg or load_config(config_path)
        self.tracker = Tracker()
        self.state = {k: {"blocked": False, "raw_blocked_s": 0.0, "raw_clear_s": 0.0}
                      for k in ("fwd", "back", "left", "right")}
        self.state["rot"] = {"blocked": False, "raw_blocked_s": 0.0, "raw_clear_s": 0.0}
        self.last_ts = None
        self.last_frame_mono = None
        self.frame_id = 0

    # ---- 迟滞: 单帧噪声不得推动整车（与 hri_state 施密特去抖同一套原则）----
    def _hysteresis(self, key, raw_blocked, gap, dt):
        st = self.state[key]
        want_clear_gap = gap_required(self.cfg["v_nom_mps"], self.cfg) + self.cfg["resume_margin_m"]
        if raw_blocked:
            st["raw_blocked_s"] += dt
            st["raw_clear_s"] = 0.0
        else:
            st["raw_clear_s"] += dt
            st["raw_blocked_s"] = 0.0
        if not st["blocked"]:
            if st["raw_blocked_s"] >= self.cfg["block_confirm_s"]:
                st["blocked"] = True
        else:
            if st["raw_clear_s"] >= self.cfg["clear_confirm_s"] and \
                    (gap is None or gap >= want_clear_gap):
                st["blocked"] = False
        return st["blocked"]

    def update(self, frame, now=None):
        cfg = self.cfg
        now = now if now is not None else time.monotonic()
        ts, raw_pts = parse_frame(frame)
        pts = to_robot_frame(raw_pts, cfg)

        dt = 0.2
        if self.last_frame_mono is not None:
            dt = min(1.0, max(0.03, now - self.last_frame_mono))
        self.last_frame_mono = now
        self.frame_id += 1

        clusters = cluster_points(pts, cfg)
        clusters = self.tracker.update(clusters, dt)
        legs = [c for c in clusters if c["cls"] == "legs"]

        # ---- 四方向 + 原地转净空 ----
        gaps = {}
        near_idx = {}
        for key, theta in (("fwd", 0.0), ("left", -90.0), ("right", 90.0), ("back", 180.0)):
            gaps[key], near_idx[key] = clearance_dir(pts, cfg, theta)
        rot_gap = rotation_gap(pts, cfg)

        # ---- 动目标提前量: 把"正在靠近"的速度折进等效间隙 ----
        lead = 0.0
        nearest = None
        for key in ("fwd", "back", "left", "right"):
            i = near_idx[key]
            if i is not None:
                cl = next((c for c in clusters if i in c["idxs"]), None)
                closing = (cl or {}).get("closing_mps", 0.0)
                if key == "fwd":
                    lead = closing * cfg["t_react_s"]
                if nearest is None or (gaps[key] is not None and gaps[key] < nearest["gap_m"]):
                    nearest = {"bearing_deg": pts[i][0], "range_m": pts[i][1],
                               "gap_m": round(gaps[key], 2),
                               "cls": (cl or {}).get("cls", "obstacle"),
                               "closing_mps": closing,
                               "dynamic": (cl or {}).get("dynamic", False)}
        if gaps["fwd"] is not None:
            gaps_eff_fwd = gaps["fwd"] - lead
        else:
            gaps_eff_fwd = None

        # ---- 速度/角速度上限 ----
        v_fwd = v_from_gap(gaps_eff_fwd if gaps_eff_fwd is not None else gaps["fwd"], cfg)
        v_back = v_from_gap(gaps["back"], cfg)
        v_left = v_from_gap(gaps["left"], cfg)
        v_right = v_from_gap(gaps["right"], cfg)
        wz_max = wz_from_gap(rot_gap, cfg)

        raw_blocked = {
            "fwd": v_fwd < cfg["v_min_mps"] or (gaps["fwd"] is not None
                                                and gaps["fwd"] < cfg["hard_gap_m"]),
            "back": v_back < cfg["v_min_mps"],
            "left": v_left < cfg["v_min_mps"],
            "right": v_right < cfg["v_min_mps"],
            "rot": wz_max < cfg["wz_min_dps"],
        }
        blocked = {}
        for key in ("fwd", "back", "left", "right"):
            blocked[key] = self._hysteresis(key, raw_blocked[key], gaps[key], dt)
        blocked["rot"] = self._hysteresis("rot", raw_blocked["rot"], rot_gap, dt)
        # 硬碰撞保护不被迟滞拖延（间距已进车体 → 立即否决）
        if gaps["fwd"] is not None and gaps["fwd"] < cfg["hard_gap_m"]:
            blocked["fwd"] = True

        # 迟滞后仍按"已确认"状态收紧速度上限（否则确认窗内还能全速走）
        if blocked["fwd"]:
            v_fwd = 0.0
        if blocked["back"]:
            v_back = 0.0
        if blocked["left"]:
            v_left = 0.0
        if blocked["right"]:
            v_right = 0.0
        if blocked["rot"]:
            wz_max = 0.0

        # ---- 绕行提示（V2 麦轮横移用; V1 只作观测）----
        escape = self._escape(blocked, gaps, v_left, v_right, v_back, cfg)

        # ---- 健康 ----
        # 三种非 ok: stale(帧过期) / suspect(有效点过少: 探头被挡、起雾、镜头脏)。
        # suspect 是重要的"安静失效"——SDK 还在按点出版本刷时间戳, 但反正什么也没看见。
        scan_age = (now - ts) if (ts is not None and now >= ts) else None
        stale = scan_age is not None and scan_age > cfg["stale_timeout_s"]
        suspect = len(pts) < cfg["min_points_ok"]
        ok = not stale and not suspect
        mode = "stale" if stale else ("suspect" if suspect else "ok")
        sec_r, sec_n = sector_minima(pts, cfg)

        env = {
            "ts": round(time.time(), 3), "mono": round(now, 3),
            "frame_id": self.frame_id,
            "ok": bool(ok),
            "mode": mode,
            "scan_age_s": None if scan_age is None else round(scan_age, 3),
            "n_points": len(pts),
            "limits": {
                "vx_max": round(v_fwd, 2), "vx_min": round(-v_back, 2),
                "vy_max": round(v_right, 2), "vy_min": round(-v_left, 2),
                "wz_max_dps": round(wz_max, 1),
            },
            "blocked": blocked,
            "gap_m": {k: (None if v is None else round(v, 2)) for k, v in gaps.items()},
            "rot_gap_m": None if rot_gap is None else round(rot_gap, 2),
            "gap_required_m": round(gap_required(cfg["v_nom_mps"], cfg), 2),
            "nearest": nearest,
            "person_legs": [{"bearing_deg": c["bearing_deg"], "range_m": c["range_m"],
                             "closing_mps": c["closing_mps"], "dynamic": c["dynamic"]}
                            for c in legs],
            "clusters": clusters,
            "sector_deg": round(360.0 / len(sec_r), 1),
            "sector_m": [None if v is None else round(v, 2) for v in sec_r],
            "sector_n": sec_n,
            "mask_deg": mask_spans_deg(cfg),
            "escape": escape,
        }
        if not ok:
            env["degrade_speed_mps"] = cfg["degrade_speed_mps"]
        return env

    def _escape(self, blocked, gaps, v_left, v_right, v_back, cfg):
        """受阻时的几何绕行提示（不做决策, 只给"往哪边走更空"）"""
        if not blocked["fwd"]:
            return None
        cands = [("left", gaps["left"], -v_left), ("right", gaps["right"], v_right),
                 ("back", gaps["back"], -v_back)]
        usable = [(d, g, v) for (d, g, v) in cands
                  if g is not None and abs(v) >= cfg["v_min_mps"]]
        if not usable:
            return {"dir": "hold", "vy_mps": 0.0, "reason": "四面受阻, 原地等待"}
        d, g, v = max(usable, key=lambda t: t[1])
        if d == "back":
            return {"dir": "back", "vy_mps": 0.0, "vx_mps": round(v, 2),
                    "reason": f"前方受阻({gaps['fwd']}), 后方净空 {g:.2f}m"}
        # 横移量按"右正左负"给符号(与 bearing 同一约定)
        return {"dir": d, "vy_mps": round(v, 2),
                "reason": f"前方受阻({gaps['fwd']}), {d}侧净空 {g:.2f}m"}


# ============================ 命令过滤（命令路径的唯一收口） ============================

def _scale_speed_token(tok, factor, cap):
    """速度 token(可带 L/R 前缀): 按比例缩放并封顶"""
    prefix = tok[0] if tok and tok[0] in "LlRr" else ""
    body = tok[1:] if prefix else tok
    try:
        v = int(float(body))
    except ValueError:
        return tok
    return f"{prefix}{int(max(0, min(cap, round(v * factor))))}"


def _spd_cap(spd_value, mps_cap, cfg):
    """PWM → 该间隙允许的 PWM(包络是 m/s, 命令是 PWM, 换算在这里收口)"""
    v_per_spd = cfg["v_at_spd_max"] / cfg["spd_max"]
    return (mps_cap / v_per_spd) if v_per_spd > 0 else spd_value


def apply_to_ascii(cmd, env, cfg=None):
    """把安全包络作用到一条 ASCII 命令上。返回 (cmd_out, note)。
    只处理运动命令（MOVE/DIFF/MEC）；STOP/PAN/TLT/TEL 原样放行。
    语义: 前向受阻 → 降为 STOP；受限 → 等比缩速（保持转向比例）；
          转向受限 → 压缩差速分量；stale → 只限速不停车。"""
    cfg = cfg or _load_defaults()
    if env is None:
        return cmd, None
    mode = env.get("mode", "ok")
    lim = env.get("limits") or {}
    blocked = env.get("blocked") or {}
    parts = cmd.split()
    if not parts:
        return cmd, None
    head = parts[0].upper()

    # ---- 降级(stale): 雷达数据不可信 → 只用"保守限速"兜底, 不施加停止 ----
    if mode != "ok":
        cap = env.get("degrade_speed_mps", cfg["degrade_speed_mps"])
        f = max(0.0, min(1.0, cap / max(1e-6, cfg["v_nom_mps"])))
        if head == "MOVE" and len(parts) == 3:
            return f"MOVE {parts[1].upper()} {_scale_speed_token(parts[2], f, 255)}", \
                f"lidar stale → 限速 {cap}m/s"
        if head == "DIFF" and len(parts) == 3:
            return " ".join(["DIFF"] + [_scale_speed_token(t, f, 255) for t in parts[1:]]), \
                f"lidar stale → 限速 {cap}m/s"
        if head == "MEC" and len(parts) == 4:
            s = cfg["mec_v_units"]
            return (f"MEC {float(parts[1]) * f:.1f} {float(parts[2]) * f:.1f} {parts[3]}",
                    f"lidar stale → 限速 {cap}m/s")
        return cmd, None

    if head == "MOVE" and len(parts) == 3:
        d, spd = parts[1].upper(), parts[2]
        if d in ("L", "R") and cfg.get("move_lr_is_rotation", True):
            # V1 固件语义: MOVE L/R = 原地旋转(坦克转向, decision/README 协议表)。
            # 因此它受"转向净空/角速度包络"约束, 而不是横向走廊包络。
            # V2 麦轮若把左右键改成"真横移", 置 move_lr_is_rotation=false 即切回走廊判定。
            if blocked.get("rot"):
                return "STOP", f"lidar: 原地转受阻(净空={env.get('rot_gap_m')})"
            cap = lim.get("wz_max_dps")
            if cap is not None and cap <= 0:
                # 还没到"确认窗"结束、但转向包络已判禁止(rot_gap 太小) → 立即停, 别转到确认
                return "STOP", f"lidar: 转向包络 0dps(净空={env.get('rot_gap_m')})"
            if cap is not None and 0 < cap < cfg["wz_max_dps"]:
                spd_c = max(0, round(float(spd) * cap / cfg["wz_max_dps"]))
                return f"MOVE {d} {int(spd_c)}", f"lidar: 转向限速 {cap}dps"
            return cmd, None
        key = {"F": "fwd", "B": "back", "L": "left", "R": "right"}.get(d)
        if key is None:
            return cmd, None
        if blocked.get(key):
            return "STOP", f"lidar: {key} 受阻(gap={env['gap_m'].get(key)})"
        cap = lim.get({"F": "vx_max", "B": "vx_min", "L": "vy_min",
                       "R": "vy_max"}[d])
        if cap is None:
            return cmd, None
        cap_spd = _spd_cap(float(spd), abs(cap), cfg)
        if cap_spd >= float(spd) - 0.5:
            return cmd, None
        return f"MOVE {d} {int(max(0, round(cap_spd)))}", "lidar: 限速"

    if head == "DIFF" and len(parts) == 3:
        try:
            l = float(parts[1].lstrip("Ll").lstrip("Rr"))
            r = float(parts[2].lstrip("Ll").lstrip("Rr"))
        except ValueError:
            return cmd, None
        avg, half = (l + r) / 2.0, (l - r) / 2.0
        if avg > 0 and blocked.get("fwd"):
            return "STOP", f"lidar: 前方受阻(gap={env['gap_m'].get('fwd')})"
        if avg < 0 and blocked.get("back"):
            return "STOP", f"lidar: 后方受阻(gap={env['gap_m'].get('back')})"
        # 前进分量按包络封顶(保持转向比例), 转向分量按转向净空压缩
        f_v = 1.0
        cap = lim.get("vx_max" if avg >= 0 else "vx_min")
        if cap is not None and abs(avg) > 0:
            cap_spd = _spd_cap(abs(avg), abs(cap), cfg)
            if cap_spd < abs(avg):
                f_v = max(0.0, cap_spd / abs(avg))
        f_t = 1.0
        if blocked.get("rot"):
            f_t = 0.0
        elif lim.get("wz_max_dps") is not None and lim["wz_max_dps"] < cfg["wz_max_dps"]:
            f_t = max(0.2, lim["wz_max_dps"] / cfg["wz_max_dps"])
        if f_v >= 0.999 and f_t >= 0.999:
            return cmd, None
        l2, r2 = avg * f_v + half * f_t, avg * f_v - half * f_t
        note = []
        if f_v < 0.999:
            note.append(f"限速×{f_v:.2f}")
        if f_t < 0.999:
            note.append(f"转向×{f_t:.2f}")
        return f"DIFF L{int(round(l2))} R{int(round(r2))}", "lidar: " + " ".join(note)

    if head == "MEC" and len(parts) == 4:
        vs, ws = cfg["mec_v_units"], cfg["mec_w_units"]
        vx, vy, wz = float(parts[1]), float(parts[2]), float(parts[3])
        vx_mps, vy_mps, wz_dps = vx / vs, vy / vs, wz / ws
        vx_c = min(vx_mps, lim.get("vx_max", 1e9)) if vx_mps >= 0 \
            else max(vx_mps, lim.get("vx_min", -1e9))
        vy_c = min(vy_mps, lim.get("vy_max", 1e9)) if vy_mps >= 0 \
            else max(vy_mps, lim.get("vy_min", -1e9))
        wz_c = max(-lim.get("wz_max_dps", 1e9), min(lim.get("wz_max_dps", 1e9), wz_dps))
        out = f"MEC {vx_c * vs:.1f} {vy_c * vs:.1f} {wz_c * ws:.3f}"
        changed = (abs(vx_c - vx_mps) > 1e-3 or abs(vy_c - vy_mps) > 1e-3
                   or abs(wz_c - wz_dps) > 1e-3)
        return out, ("lidar: 包络钳位" if changed else None)
    return cmd, None


def grid_sensors(env):
    """§14.4 的 grid.json sensors.lidar 片段（供 geometry_daemon 合入, 消费方零改动）"""
    if env is None:
        return {"lidar": {"ok": False, "age_s": None, "mode": "off"}}
    return {"lidar": {"ok": bool(env.get("ok")), "age_s": env.get("scan_age_s"),
                      "mode": env.get("mode", "off"),
                      "n_points": env.get("n_points")}}


def read_safety(path="/dev/shm/lidar_safety.json", max_age_s=None):
    """读一份安全包络（消费方容错: 缺失/半截/过期 → None）"""
    try:
        with open(path, encoding="utf-8") as f:
            env = json.load(f)
    except Exception:
        return None
    if max_age_s is not None:
        age = time.time() - float(env.get("ts", 0))
        if age > max_age_s:
            return {"ok": False, "mode": "stale", "scan_age_s": round(age, 2),
                    "limits": {}, "blocked": {}, "gap_m": {}}
    return env


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    cfg = load_config()
    frame = json.load(open(argv[0], encoding="utf-8"))
    guard = Guard(cfg)
    env = guard.update(frame)
    print(json.dumps(env, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
