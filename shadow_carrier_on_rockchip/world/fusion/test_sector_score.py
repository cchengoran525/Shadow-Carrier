#!/usr/bin/env python3
"""test_sector_score.py —— 打分器回归夹具（离线，无需硬件）
把 2026-10-11 审核发现固化为断言；改权重/规则后必须全绿。
用法: python3 test_sector_score.py
"""
import sys
import sector_score as ss

PASS = []
FAIL = []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    print("%-4s %s %s" % ("PASS" if cond else "FAIL", name, detail if not cond else ""))


def S(**kw):
    return ss.score_map(**kw)


# T1 宿舍基线：门 45°、自由 7/0、冰箱 -30°
r = S(map_summary={"doors": [{"bearing_deg": 45, "age_s": 1}],
                   "free_sectors": [{"sector": 7, "age_s": 1}, {"sector": 0, "age_s": 1}],
                   "objects": [{"cls": "refrigerator", "bearing_deg": -30, "age_s": 1}]})
check("T1 门被罚/最优在自由扇区", r["best_sector"] in (7, 0) and r["masked_sectors"] == [], str(r["best_sector"]))

# T2 两门正对：不得出现"路中间"随机落点
r = S(map_summary={"doors": [{"bearing_deg": 0, "age_s": 1}, {"bearing_deg": 180, "age_s": 1}]})
check("T2 反向门对不产生 path 惩罚", r["reasons"]["path"] == [], str(r["reasons"]["path"]))

# T2b 90° 相隔的两门：应产生 path 惩罚（中点 45° 或 135°）
r = S(map_summary={"doors": [{"bearing_deg": 0, "age_s": 1}, {"bearing_deg": 90, "age_s": 1}]})
check("T2b 相邻两门仍产生 path", len(r["reasons"]["path"]) >= 1, str(r["reasons"]["path"]))

# T3 全扇区硬禁区 → best=None, fallback_masked=True（不得返回厕所扇区）
r = S(map_summary={"objects": [{"cls": "toilet", "bearing_deg": b, "age_s": 1} for b in range(0, 360, 45)]})
check("T3 全 mask → best=None/fallback", r["best_sector"] is None and r["fallback_masked"] and not r["confident"])

# T4 类别洪泛：CLASS_W 物体不得进 path；最优扇区不被这些惩罚淹没
r = S(map_summary={"objects": [{"cls": c, "bearing_deg": b, "age_s": 1} for c, b in
                               zip(["bed", "chair", "dining table", "laptop", "tv"], [0, 45, 90, 135, 180])]})
check("T4 类别不进 path（修洪泛）", r["reasons"]["path"] == [], str(r["reasons"]["path"]))
check("T4b 类别独立惩罚仍在", len(r["reasons"]["class_w"]) >= 3, str(r["reasons"]["class_w"]))

# T5 背墙加成：需 grid_polar（近环 free + 中/远环 blocked）
polar_wall = [[{"label": "free"} for _ in range(8)] for _ in range(3)]
for s in range(8):                      # 3 号扇区背后有墙
    polar_wall[1][s] = {"label": "blocked"}
pol_wall = [row[:] for row in polar_wall]
for r_i in range(3):
    for s in range(8):
        pol_wall[r_i][s] = {"label": "free"}
pol_wall[1][3] = {"label": "blocked"}
r = S(map_summary={"free_sectors": [{"sector": 3, "ring": 0, "age_s": 1}]}, grid_polar=pol_wall)
check("T5 背后有墙才有 wall_back", r["reasons"]["wall_back"] == [3], str(r["reasons"]["wall_back"]))
r = S(map_summary={"free_sectors": [{"sector": 3, "ring": 0, "age_s": 1}]})   # 无 grid_polar
check("T5b 无 grid_polar 不加 wall_back", r["reasons"]["wall_back"] == [], str(r["reasons"]["wall_back"]))

# T6 margin：两个并列自由扇区 → margin=0 → confident=False
r = S(map_summary={"free_sectors": [{"sector": 3, "ring": 0, "age_s": 1},
                                    {"sector": 5, "ring": 0, "age_s": 1}]})
check("T6 并列无 margin → 不 confident", (r["margin"] == 0.0) and not r["confident"], str(r))

# T7 blind 不落 mask
pol = [[{"label": "free"} for _ in range(8)] for _ in range(3)]
r = S(map_summary={"objects": [{"cls": "toilet", "bearing_deg": 180, "age_s": 1}],
                   "persons": [{"bearing_deg": 0, "facing_deg": 0, "age_s": 1}]})
check("T7 blind 不落 mask 扇区", 4 not in r["reasons"]["blind"], str(r["reasons"]["blind"]))

# T8 门状态化：ajar 比 open 罚得更重
a = S(map_summary={"doors": [{"bearing_deg": 0, "state": "ajar", "age_s": 1}]})["scores"][0]
o = S(map_summary={"doors": [{"bearing_deg": 0, "state": "open", "age_s": 1}]})["scores"][0]
check("T8 ajar 罚重于 open", a < o, "ajar=%s open=%s" % (a, o))

# T9 环权重：近环 free 分值高于中环
n = S(map_summary={"free_sectors": [{"sector": 2, "ring": 0, "age_s": 1}]})["scores"][2]
m = S(map_summary={"free_sectors": [{"sector": 2, "ring": 1, "age_s": 1}]})["scores"][2]
check("T9 近环 > 中环", n > m, "near=%s mid=%s" % (n, m))

# T10 过期数据不入分
r = S(map_summary={"doors": [{"bearing_deg": 0, "age_s": 99}],
                   "free_sectors": [{"sector": 3, "age_s": 99}]})
check("T10 过期数据被忽略", r["reasons"]["door_pen"] == [] and r["reasons"]["free"] == [])

print("\n== %d passed, %d failed ==" % (len(PASS), len(FAIL)))
sys.exit(1 if FAIL else 0)
