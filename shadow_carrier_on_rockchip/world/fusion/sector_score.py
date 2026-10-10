#!/usr/bin/env python3
"""
sector_score.py —— HIDE 选点"交互亲和度"打分（纯函数，可离线回归）
对应主 README §HIDE 选点语义。

v2 修复（2026-10-11，[世界] 审核后，逐条对应审核发现）:
  A 反向热点对退化     → 圆形均值加"合成向量模长"守卫, 近反向对跳过
  B 全 mask 仍返回 best → best_sector=None + fallback_masked=True
  C 类别洪泛污染路中间  → path 只吃 门+家电(真交通节点), CLASS_W 独立惩罚
  D wall_back 语义错    → 改为「近环 free + 中/远环 blocked = 背后有墙」, 依赖 grid_polar;
                          无 grid_polar 时不加成（不再把开阔当背墙）
  E 缺 margin          → 暴露 margin(best-second), confident 要求 margin≥MARGIN_MIN
  F blind 撞 mask      → blind 加成跳过 masked 扇区; mask 分数按扇区去重

输入(全部可选除 doors/free_sectors): 
  {"doors":[{bearing_deg,age_s,state?}],
   "free_sectors":[{sector,ring?,age_s}],          # ring 0/1/2, 缺省 0
   "objects":[{cls,bearing_deg,age_s}],
   "persons":[{bearing_deg,facing_deg?,age_s}],
   "hour_bucket":"default|peak|night"}
  grid_polar: 3x8 标签矩阵(可选, 用于背墙判定) —— 由 geometry_daemon 传入

输出: {scores[8], best_sector|None, best_bearing_deg|None, best_score, margin,
       confident, fallback_masked, ranking[[sector,score]...], masked_sectors[], reasons{}}
"""
import math

N_SECTORS = 8
SECTOR_DEG = 45.0
MARGIN_MIN = 0.20         # best 与次优的最小间距（confident 门槛）
OPPOSITE_GUARD = 0.25     # 热点对合成向量模长下限（低于=近反向, 跳过）

W = {
    "free_near": 1.0,     # 自由扇区加成（近环）
    "free_mid": 0.6,      # 中环
    "free_far": 0.3,      # 远环
    "door": -1.6,         # 门（缺省, 无 state 时）
    "door_cone": -0.9,    # 门进出锥
    "appliance": -0.8,    # 冰箱/饮水机前方
    "path": -0.6,         # 门/家电两两连线中点 = 路中间
    "wall_back": 0.7,     # 背墙加成（近环 free + 中/远环 blocked）
    "gaze": -0.5,         # 人面向扇区
    "blind": 0.3,         # 人背后扇区
}

APPLIANCE_CLASSES = {"refrigerator", "vending machine", "water dispenser",
                     "sink", "microwave", "oven"}
MAX_AGE_S = 30.0

CLASS_W = {
    "bed": -1.0, "couch": -0.7, "chair": -0.5,
    "dining table": -0.6, "bench": -0.5,
    "tv": -0.4, "laptop": -0.4,
    "potted plant": -0.3, "vase": -0.3,
    "cash register": -1.0, "fitting room": -1.0, "shop entrance": -0.9,
    "queue": -1.2, "kiosk": -0.8, "staircase": -1.8,
    "display stand": -0.6, "playground": -0.7, "pet": -0.5,
}

HARD_MASK_CLASSES = {"toilet", "bathroom door", "escalator", "fire extinguisher",
                     "first aid kit", "electrical panel", "fire exit"}
MASK_W = -99.0

DOOR_STATE_W = {"ajar": -2.2, "closed": -1.6, "open": -1.0}
PENALTY_SCALE = {"default": 1.0, "peak": 1.25, "night": 1.5}
FREE_W_BY_RING = {0: W["free_near"], 1: W["free_mid"], 2: W["free_far"]}


def _sector_of(bearing):
    return int(((bearing + SECTOR_DEG / 2) % 360) // SECTOR_DEG) % N_SECTORS


def _sector_bearing(s):
    return (s * SECTOR_DEG + 180) % 360 - 180


def _angle_diff(a, b):
    return abs(((a - b + 180) % 360) - 180)


def _is_blocked(label):
    lab = (label or {}).get("label", "") if isinstance(label, dict) else str(label)
    return lab == "blocked" or lab.startswith("region:") or lab.startswith("obj:")


def _is_free(label):
    lab = (label or {}).get("label", "") if isinstance(label, dict) else str(label)
    return lab == "free"


def score_map(map_summary, grid_polar=None):
    scores = [0.0] * N_SECTORS
    reasons = {"free": [], "door_pen": [], "appliance": [], "path": [], "wall_back": [],
               "class_w": [], "gaze": [], "blind": [], "mask": []}
    scale = PENALTY_SCALE.get(map_summary.get("hour_bucket", "default"), 1.0)

    def _neg(w):
        return w * scale if w < 0 else w

    objects = [o for o in (map_summary.get("objects") or []) if o.get("age_s", 99) <= MAX_AGE_S]

    # ---- 硬禁区: 每扇区只记一次, 从候选剔除 ----
    masked = set()
    for o in objects:
        if o.get("cls") in HARD_MASK_CLASSES:
            s = _sector_of(o.get("bearing_deg", 0))
            if s not in masked:
                masked.add(s)
                scores[s] += MASK_W
                reasons["mask"].append(s)

    # ---- 类别权重 / 家电（mask 优先, 只加一次）----
    appliance_secs = []
    for o in objects:
        s = _sector_of(o.get("bearing_deg", 0))
        if s in masked:
            continue
        if o.get("cls") in APPLIANCE_CLASSES:
            scores[s] += _neg(W["appliance"])
            reasons["appliance"].append(s)
            appliance_secs.append(s)
        elif o.get("cls") in CLASS_W:
            scores[s] += _neg(CLASS_W[o["cls"]])
            reasons["class_w"].append(s)

    # ---- 门（状态化）+ 进出锥 ----
    door_secs = []
    for d in map_summary.get("doors", []):
        if d.get("age_s", 99) > MAX_AGE_S:
            continue
        s = _sector_of(d.get("bearing_deg", 0))
        door_secs.append(s)
        scores[s] += _neg(DOOR_STATE_W.get(d.get("state"), W["door"]))
        reasons["door_pen"].append(s)
        for ds in (-1, 1):
            n = (s + ds) % N_SECTORS
            scores[n] += _neg(W["door_cone"])
            reasons["door_pen"].append(n)

    # ---- 自由扇区（按环加权）----
    free_secs = set()
    near_free = set()
    for f in map_summary.get("free_sectors", []):
        if f.get("age_s", 99) > MAX_AGE_S:
            continue
        s = int(f.get("sector", _sector_of(f.get("bearing_deg", 0))))
        ring = int(f.get("ring", 0))
        if s in masked:
            continue
        scores[s] += FREE_W_BY_RING.get(ring, W["free_near"])
        free_secs.add(s)
        if ring == 0:
            near_free.add(s)
        reasons["free"].append(s)

    # ---- 朝向锥 ----
    for p in (map_summary.get("persons") or []):
        if p.get("age_s", 99) > MAX_AGE_S or p.get("facing_deg") is None:
            continue
        fs = _sector_of(p.get("facing_deg"))
        if fs not in masked:
            scores[fs] += _neg(W["gaze"])
            reasons["gaze"].append(fs)
        bs = (fs + N_SECTORS // 2) % N_SECTORS
        if bs not in masked:                      # F: blind 不落 mask
            scores[bs] += W["blind"]
            reasons["blind"].append(bs)

    # ---- 背墙加成（D: 需 grid_polar; 近环 free + 中/远环 blocked）----
    if grid_polar:
        for s in range(N_SECTORS):
            if s in masked or s not in near_free:
                continue
            behind = any(_is_blocked(grid_polar[r][s]) for r in (1, 2) if r < len(grid_polar))
            if behind and s not in reasons["door_pen"] and s not in reasons["gaze"]:
                scores[s] += W["wall_back"]
                reasons["wall_back"].append(s)

    # ---- 路中间（C: 只吃 门+家电; A: 反向对跳过）----
    hotspots = sorted(set(door_secs + appliance_secs))
    for i in range(len(hotspots)):
        for j in range(i + 1, len(hotspots)):
            a, b = _sector_bearing(hotspots[i]), _sector_bearing(hotspots[j])
            if _angle_diff(a, b) < SECTOR_DEG:
                continue
            ar, br = math.radians(a), math.radians(b)
            vx, vy = (math.cos(ar) + math.cos(br)) / 2, (math.sin(ar) + math.sin(br)) / 2
            if math.hypot(vx, vy) < OPPOSITE_GUARD:      # A: 近反向对, 中点无意义
                continue
            ms = _sector_of(math.degrees(math.atan2(vy, vx)))
            if ms in masked:
                continue
            scores[ms] += _neg(W["path"])
            reasons["path"].append(ms)

    # ---- 候选与排序 ----
    cands = [s for s in range(N_SECTORS) if s not in masked]
    fallback_masked = not cands
    if fallback_masked:
        best = None
        best_score = None
        margin = None
        ranking = []
    else:
        ranking = sorted(cands, key=lambda s: scores[s], reverse=True)
        best = ranking[0]
        best_score = round(scores[best], 2)
        second = scores[ranking[1]] if len(ranking) > 1 else None
        margin = round(best_score - second, 2) if second is not None else None

    confident = (best is not None and bool(free_secs) and scores[best] > 0.0
                 and (margin is None or margin >= MARGIN_MIN))

    return {"scores": [round(x, 2) for x in scores],
            "best_sector": best,
            "best_bearing_deg": _sector_bearing(best) if best is not None else None,
            "best_score": best_score,
            "margin": margin,
            "confident": confident,
            "fallback_masked": fallback_masked,
            "ranking": [[s, round(scores[s], 2)] for s in ranking],
            "masked_sectors": sorted(masked),
            "reasons": {k: sorted(set(v)) for k, v in reasons.items()}}


if __name__ == "__main__":
    demo = {"doors": [{"bearing_deg": 45, "age_s": 1}],
            "free_sectors": [{"sector": 7, "ring": 0, "age_s": 1}, {"sector": 0, "ring": 0, "age_s": 1}],
            "objects": [{"cls": "refrigerator", "bearing_deg": -30, "age_s": 1}]}
    print("dorm:", score_map(demo))
