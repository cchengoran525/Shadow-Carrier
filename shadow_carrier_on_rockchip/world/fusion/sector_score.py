#!/usr/bin/env python3
"""
sector_score.py —— HIDE 选点"交互亲和度"打分（纯函数，可离线回归）
对应主 README §HIDE 选点语义：门/走道口强惩罚(含进出锥)、冰箱饮水机惩罚前方扇区、
两热点连线=路中间惩罚、墙角/背墙加成；自由扇区正向加成。

输入: 滚动地图 summary {doors[], free_sectors[], objects?}（或 grid.json 的快照）
输出: {"scores":[8], "best_sector":int, "best_bearing_deg":float, "best_score":float,
       "reasons":{"free":[...], "door_pen":[...], ...}}

设计约束（与 [HRI] 约定）:
  - 纯函数：无 IO、无时间依赖（age 由调用方传入的字段决定）
  - 接口纯增量：grid.json 新增 "sector_scores" 字段，不改既有字段
  - 权重集中 W 字典，便于按 demo 场景调
"""
import math

N_SECTORS = 8
SECTOR_DEG = 45.0

# 权重（惩罚为负、加成为正）——集中可调
W = {
    "free": 1.0,          # 自由扇区加成
    "door": -1.6,         # 门/走道口强惩罚
    "door_cone": -0.9,    # 门进出锥（相邻扇区）
    "appliance": -0.8,    # 冰箱/饮水机前方扇区惩罚
    "path": -0.6,         # 两热点连线（路中间）
    "wall_back": 0.7,     # 背墙/墙角加成（中远环阻塞 + 近环自由）
}

APPLIANCE_CLASSES = {"refrigerator", "vending machine", "water dispenser",
                     "sink", "microwave", "oven"}
MAX_AGE_S = 30.0          # 与滚动地图 TTL 一致


def _sector_of(bearing):
    return int(((bearing + SECTOR_DEG / 2) % 360) // SECTOR_DEG) % N_SECTORS


def _sector_bearing(s):
    return (s * SECTOR_DEG + 180) % 360 - 180   # 0,45,... 归一到 [-180,180)


def _angle_diff(a, b):
    return abs(((a - b + 180) % 360) - 180)


def score_map(map_summary):
    """map_summary: {"doors":[{bearing_deg,age_s}], "free_sectors":[{sector,age_s}],
                     "objects":[{cls,bearing_deg,age_s}] (可选)}"""
    scores = [0.0] * N_SECTORS
    reasons = {"free": [], "door_pen": [], "appliance": [], "path": [], "wall_back": []}

    free_secs = set()
    for f in map_summary.get("free_sectors", []):
        if f.get("age_s", 99) <= MAX_AGE_S:
            s = int(f.get("sector", _sector_of(f.get("bearing_deg", 0))))
            scores[s] += W["free"]
            free_secs.add(s)
            reasons["free"].append(s)

    door_secs = []
    for d in map_summary.get("doors", []):
        if d.get("age_s", 99) > MAX_AGE_S:
            continue
        b = d.get("bearing_deg", 0)
        s = _sector_of(b)
        door_secs.append(s)
        scores[s] += W["door"]
        reasons["door_pen"].append(s)
        # 进出锥: 门所在扇区的左右邻域也罚（人从门进出会经过）
        for ds in (-1, 1):
            n = (s + ds) % N_SECTORS
            scores[n] += W["door_cone"]
            reasons["door_pen"].append(n)

    for o in map_summary.get("objects", []) or []:
        if o.get("age_s", 99) > MAX_AGE_S:
            continue
        if o.get("cls") in APPLIANCE_CLASSES:
            s = _sector_of(o.get("bearing_deg", 0))
            scores[s] += W["appliance"]
            reasons["appliance"].append(s)

    # 两热点连线（门/家电两两之间）近似"路中间"
    hotspots = door_secs + reasons["appliance"]
    if len(hotspots) >= 2:
        hs = sorted(set(hotspots))
        for i in range(len(hs)):
            for j in range(i + 1, len(hs)):
                a, b = _sector_bearing(hs[i]), _sector_bearing(hs[j])
                if _angle_diff(a, b) < SECTOR_DEG:
                    continue
                mid = math.degrees(math.atan2(
                    (math.sin(math.radians(a)) + math.sin(math.radians(b))) / 2,
                    (math.cos(math.radians(a)) + math.cos(math.radians(b))) / 2))
                ms = _sector_of(mid)
                scores[ms] += W["path"]
                reasons["path"].append(ms)

    # 背墙加成: 用"自由扇区"近似可得位置（有自由=可停靠的靠墙位）
    for s in free_secs:
        if s not in reasons["door_pen"]:
            scores[s] += W["wall_back"]
            reasons["wall_back"].append(s)

    best = max(range(N_SECTORS), key=lambda s: scores[s])
    return {"scores": [round(x, 2) for x in scores],
            "best_sector": best,
            "best_bearing_deg": _sector_bearing(best),
            "best_score": round(scores[best], 2),
            "confident": bool(free_secs) and scores[best] > 0.0,
            "reasons": {k: sorted(set(v)) for k, v in reasons.items()}}


if __name__ == "__main__":
    # 自测: 门在 45°, 自由扇区 7 和 0
    demo = {"doors": [{"bearing_deg": 45, "age_s": 1}],
            "free_sectors": [{"sector": 7, "age_s": 1}, {"sector": 0, "age_s": 1}],
            "objects": [{"cls": "refrigerator", "bearing_deg": -30, "age_s": 1}]}
    r = score_map(demo)
    print("scores:", r["scores"])
    print("best:", r["best_sector"], r["best_bearing_deg"], "score", r["best_score"])
    print("reasons:", r["reasons"])
