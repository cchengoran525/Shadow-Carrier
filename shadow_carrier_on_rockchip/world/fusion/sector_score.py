#!/usr/bin/env python3
"""
sector_score.py —— HIDE 选点"交互亲和度"打分（纯函数，可离线回归）
对应主 README §HIDE 选点语义：门/走道口强惩罚(含进出锥)、冰箱饮水机惩罚前方扇区、
两热点连线=路中间惩罚、墙角/背墙加成；自由扇区正向加成。

输入: 滚动地图 summary {doors[], free_sectors[], objects?}（或 grid.json 的快照）
输出: {"scores":[8], "best_sector":int, "best_bearing_deg":float, "best_score":float,
       "confident":bool, "masked_sectors":[int], "reasons":{...}}

增量规则（2026-10-07，见 docs/AFFINITY_SCORE_LITERATURE.md §4；新字段缺席时与旧版
逐字节一致，demo 冻结期安全）:
  - CLASS_W            类别→权重表（YOLO-World 开放词汇直接喂字符串）:
                       驻留预测(床/沙发/桌椅)、易碎(盆栽)、商场(收银/队列/中岛…)
  - HARD_MASK_CLASSES  硬禁区(toilet/扶梯/灭火器…): 负分不够, 直接剔除出 best 候选
                       —— 安全=硬约束, 社交=软分数
  - DOOR_STATE_W       门状态化: ajar(-2.2) > closed(-1.6) > open(-1.0), 打分对象是
                       "不可预测性"而非门本身; 缺省回退 W["door"] 旧常数
  - persons[].facing_deg  朝向锥: 人面向扇区罚(gaze), 人背后扇区赏(blind)
  - hour_bucket        时段桶(default/peak/night): 负权重放大; 纯函数不取系统时间,
                       由调用方传入（同 age_s 模式）

设计约束（与 [HRI] 约定）:
  - 纯函数：无 IO、无时间依赖（age/hour 由调用方传入的字段决定）
  - 接口纯增量：grid.json 新增 "sector_scores" 字段，不改既有字段
  - 权重集中 W/CLASS_W/DOOR_STATE_W 字典，便于按 demo 场景调
"""
import math

N_SECTORS = 8
SECTOR_DEG = 45.0

# 权重（惩罚为负、加成为正）——集中可调
W = {
    "free": 1.0,          # 自由扇区加成
    "door": -1.6,         # 门/走道口强惩罚（无状态时的缺省）
    "door_cone": -0.9,    # 门进出锥（相邻扇区）
    "appliance": -0.8,    # 冰箱/饮水机前方扇区惩罚
    "path": -0.6,         # 两热点连线（路中间）
    "wall_back": 0.7,     # 背墙/墙角加成（中远环阻塞 + 近环自由）
    "gaze": -0.5,         # 人面向扇区（被注视=被注意, Goffman civil inattention 反面）
    "blind": 0.3,         # 人背后扇区（不被注视的藏点）
}

APPLIANCE_CLASSES = {"refrigerator", "vending machine", "water dispenser",
                     "sink", "microwave", "oven"}
MAX_AGE_S = 30.0          # 与滚动地图 TTL 一致

# 类别→权重表：手写规则的接口面。YOLO-World 开放词汇直接喂字符串,
# 宿舍包/商场包 = 两份表, 本函数零改动（docs/AFFINITY_SCORE_LITERATURE.md §3）
CLASS_W = {
    # 驻留预测: 人即将出现的位置（预测性惩罚, 而非当前占用）
    "bed": -1.0, "couch": -0.7, "chair": -0.5,
    "dining table": -0.6, "bench": -0.5,
    # 注视/交互区
    "tv": -0.4, "laptop": -0.4,
    # 易碎/财产剐蹭
    "potted plant": -0.3, "vase": -0.3,
    # 逛街/商场场景
    "cash register": -1.0, "fitting room": -1.0, "shop entrance": -0.9,
    "queue": -1.2, "kiosk": -0.8, "staircase": -1.8,
    "display stand": -0.6, "playground": -0.7, "pet": -0.5,
}

# 硬禁区: 隐私/法规/物理安全——从 best 候选剔除（负分不够）
HARD_MASK_CLASSES = {"toilet", "bathroom door", "escalator", "fire extinguisher",
                     "first aid kit", "electrical panel", "fire exit"}
MASK_W = -99.0            # 仅用于可视化/可解释, mask 扇区不参与 best

# 门状态→权重: 半开最不可预测; 敞开反而可见可预测
DOOR_STATE_W = {"ajar": -2.2, "closed": -1.6, "open": -1.0}

# 时段负权重放大（调用方传 hour_bucket; 加成不变）
PENALTY_SCALE = {"default": 1.0, "peak": 1.25, "night": 1.5}


def _sector_of(bearing):
    return int(((bearing + SECTOR_DEG / 2) % 360) // SECTOR_DEG) % N_SECTORS


def _sector_bearing(s):
    return (s * SECTOR_DEG + 180) % 360 - 180   # 0,45,... 归一到 [-180,180)


def _angle_diff(a, b):
    return abs(((a - b + 180) % 360) - 180)


def score_map(map_summary):
    """map_summary: {"doors":[{bearing_deg,age_s,state?}], "free_sectors":[{sector,age_s}],
                     "objects":[{cls,bearing_deg,age_s}] (可选),
                     "persons":[{bearing_deg,facing_deg?,age_s}] (可选),
                     "hour_bucket": "default"|"peak"|"night" (可选)}"""
    scores = [0.0] * N_SECTORS
    reasons = {"free": [], "door_pen": [], "appliance": [], "path": [], "wall_back": [],
               "class_w": [], "gaze": [], "blind": [], "mask": []}
    scale = PENALTY_SCALE.get(map_summary.get("hour_bucket", "default"), 1.0)

    def _neg(w):
        return w * scale if w < 0 else w   # 负权重随时段放大, 加成不变

    # 预扫: 硬禁区类别 → mask（后续加成跳过这些扇区）
    masked = set()
    for o in map_summary.get("objects", []) or []:
        if o.get("age_s", 99) <= MAX_AGE_S and o.get("cls") in HARD_MASK_CLASSES:
            masked.add(_sector_of(o.get("bearing_deg", 0)))

    # 类别权重（mask 优先于普通类别）
    for o in map_summary.get("objects", []) or []:
        if o.get("age_s", 99) > MAX_AGE_S:
            continue
        s = _sector_of(o.get("bearing_deg", 0))
        if s in masked:
            scores[s] += MASK_W
            reasons["mask"].append(s)
        elif o.get("cls") in APPLIANCE_CLASSES:
            scores[s] += _neg(W["appliance"])
            reasons["appliance"].append(s)
        elif o.get("cls") in CLASS_W:
            scores[s] += _neg(CLASS_W[o["cls"]])
            reasons["class_w"].append(s)

    door_secs = []
    for d in map_summary.get("doors", []):
        if d.get("age_s", 99) > MAX_AGE_S:
            continue
        b = d.get("bearing_deg", 0)
        s = _sector_of(b)
        door_secs.append(s)
        # 门状态: 打分对象是"不可预测性"(半开 > 关 > 敞开), 缺省回退旧常数
        scores[s] += _neg(DOOR_STATE_W.get(d.get("state"), W["door"]))
        reasons["door_pen"].append(s)
        # 进出锥: 门所在扇区的左右邻域也罚（人从门进出会经过）
        for ds in (-1, 1):
            n = (s + ds) % N_SECTORS
            scores[n] += _neg(W["door_cone"])
            reasons["door_pen"].append(n)

    free_secs = set()
    for f in map_summary.get("free_sectors", []):
        if f.get("age_s", 99) <= MAX_AGE_S:
            s = int(f.get("sector", _sector_of(f.get("bearing_deg", 0))))
            if s not in masked:      # 硬禁区不吃自由加成
                scores[s] += W["free"]
                free_secs.add(s)
                reasons["free"].append(s)

    # 朝向锥: 人面向扇区罚, 人背后扇区赏（直径相对扇区 = +180°）
    for p in map_summary.get("persons", []) or []:
        if p.get("age_s", 99) > MAX_AGE_S or p.get("facing_deg") is None:
            continue
        fs = _sector_of(p.get("facing_deg"))
        scores[fs] += _neg(W["gaze"])
        reasons["gaze"].append(fs)
        bs = (fs + N_SECTORS // 2) % N_SECTORS
        scores[bs] += W["blind"]
        reasons["blind"].append(bs)

    # 两热点连线（门/家电/加权类别两两之间）近似"路中间"
    hotspots = door_secs + reasons["appliance"] + reasons["class_w"]
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
                scores[ms] += _neg(W["path"])
                reasons["path"].append(ms)

    # 背墙加成: 用"自由扇区"近似可得位置（有自由=可停靠的靠墙位）
    for s in free_secs:
        if s not in reasons["door_pen"] and s not in masked and s not in reasons["gaze"]:
            scores[s] += W["wall_back"]
            reasons["wall_back"].append(s)

    cands = [s for s in range(N_SECTORS) if s not in masked] or list(range(N_SECTORS))
    best = max(cands, key=lambda s: scores[s])
    return {"scores": [round(x, 2) for x in scores],
            "best_sector": best,
            "best_bearing_deg": _sector_bearing(best),
            "best_score": round(scores[best], 2),
            "confident": bool(free_secs) and scores[best] > 0.0 and best not in masked,
            "masked_sectors": sorted(masked),
            "reasons": {k: sorted(set(v)) for k, v in reasons.items()}}


if __name__ == "__main__":
    # 自测1(回归): 门在 45°, 自由扇区 7 和 0 —— 与旧版期望一致
    demo = {"doors": [{"bearing_deg": 45, "age_s": 1}],
            "free_sectors": [{"sector": 7, "age_s": 1}, {"sector": 0, "age_s": 1}],
            "objects": [{"cls": "refrigerator", "bearing_deg": -30, "age_s": 1}]}
    r = score_map(demo)
    print("dorm scores:", r["scores"])
    print("dorm best:", r["best_sector"], r["best_bearing_deg"], "score", r["best_score"])
    print("dorm reasons:", r["reasons"])

    # 自测2(增量): 商场场景 —— 扶梯(mask)+半开门+收银台+盆栽+人朝向+高峰时段
    mall = {"doors": [{"bearing_deg": 90, "age_s": 1, "state": "ajar"}],
            "free_sectors": [{"sector": 6, "age_s": 1}],
            "objects": [{"cls": "escalator", "bearing_deg": 0, "age_s": 1},
                        {"cls": "cash register", "bearing_deg": 180, "age_s": 1},
                        {"cls": "potted plant", "bearing_deg": -120, "age_s": 1}],
            "persons": [{"bearing_deg": 0, "facing_deg": 90, "age_s": 1}],
            "hour_bucket": "peak"}
    m = score_map(mall)
    print("mall scores:", m["scores"])
    print("mall best:", m["best_sector"], m["best_bearing_deg"], "score", m["best_score"])
    print("mall masked:", m["masked_sectors"], "confident:", m["confident"])
    print("mall reasons:", m["reasons"])
