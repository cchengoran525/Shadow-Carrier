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

# T11 person 负权重（hide_occupied FAIL 修复）：被占扇区不得成为最优
pol = [[{"label": "free"} for _ in range(8)] for _ in range(3)]
pol[1][2] = {"label": "blocked"}          # 2 号扇区背后有墙（+wall_back）
occ = {"free_sectors": [{"sector": 2, "ring": 0, "age_s": 1},
                        {"sector": 6, "ring": 0, "age_s": 1}],
       "objects": [{"cls": "person", "bearing_deg": 90, "age_s": 1}]}   # 人站在 2 号
r = S(map_summary=occ, grid_polar=pol)
check("T11 被占扇区有 person 惩罚", r["reasons"]["class_w"] == [2], str(r["reasons"]))
check("T11b 最优避开被占扇区", r["best_sector"] != 2, "scores=%s best=%s" % (r["scores"], r["best_sector"]))
# 量级校验：被占扇区 = free(1.0)+wall_back(0.7)+person(-1.2) = 0.5 < 对照 1.0
check("T11c 被占扇区分数低于干净自由扇区", r["scores"][2] < r["scores"][6],
      "occ=%s clean=%s" % (r["scores"][2], r["scores"][6]))

# T12 同扇区多人叠加（越挤越差）
r1 = S(map_summary={"objects": [{"cls": "person", "bearing_deg": 90, "age_s": 1}]})
r2 = S(map_summary={"objects": [{"cls": "person", "bearing_deg": 80, "age_s": 1},
                                {"cls": "person", "bearing_deg": 100, "age_s": 1}]})
check("T12 同扇区多人叠加更差", r2["scores"][2] < r1["scores"][2],
      "1人=%s 2人=%s" % (r1["scores"][2], r2["scores"][2]))

# T13 唯一自由证据在被占扇区、且无干净替代 → 不给 confident（宁缺毋滥）
pol2 = [[{"label": "free"} for _ in range(8)] for _ in range(3)]
pol2[1][2] = {"label": "blocked"}      # 2 号背后有墙 → 1.7 分
r = S(map_summary={"free_sectors": [{"sector": 2, "ring": 0, "age_s": 1}],
                   "objects": [{"cls": "person", "bearing_deg": 90, "age_s": 1}]},
      grid_polar=pol2)
check("T13 被占且无干净替代 → 不 confident",
      r["best_occupied"] and not r["confident"] and r["best_score"] is not None and r["best_score"] > 0,
      str(r))

# T14 hide_occupied 走廊复现（10-13）：自由点与"饮水机+冰箱"同扇区 → 被正确拒绝
# 坐标来自 Webots 场景实测: robot(-0.23,-2.46) yaw0; 饮水机 bearing 4.0°, 冰箱 -16.0°,
# HIDE_SPOT 33.8°, SPOT_B(新) -16.8°(=冰箱方向!) → sector0 双家电 -1.6, 全场无正分
corridor = {"doors": [{"bearing_deg": -168.9, "age_s": 1}, {"bearing_deg": 119.3, "age_s": 1}],
            "free_sectors": [{"bearing_deg": 33.8, "age_s": 1}, {"bearing_deg": -16.8, "age_s": 1}],
            "objects": [{"cls": "door", "bearing_deg": -168.9, "age_s": 1},
                        {"cls": "door", "bearing_deg": 119.3, "age_s": 1},
                        {"cls": "water dispenser", "bearing_deg": 4.0, "age_s": 1},
                        {"cls": "refrigerator", "bearing_deg": -16.0, "age_s": 1},
                        {"cls": "person", "bearing_deg": 33.8, "age_s": 1}]}
r = S(map_summary=corridor)
check("T14 自由点与双家电同扇区 → 拒绝 confident（防'在冰箱前躲'）",
      (not r["confident"]) and r["best_score"] <= 0.0,
      "scores=%s best=%s conf=%s" % (r["scores"], r["best_sector"], r["confident"]))
# T14b 同走廊，但自由点落在无物体扇区（-45°=sector7）→ 应 confident
corridor_ok = dict(corridor)
corridor_ok["free_sectors"] = [{"bearing_deg": 33.8, "age_s": 1}, {"bearing_deg": -45.0, "age_s": 1}]
r = S(map_summary=corridor_ok)
check("T14b 自由点落在干净扇区 → confident 且选中它",
      r["confident"] and r["best_bearing_deg"] == -45.0,
      "scores=%s best=%s conf=%s" % (r["scores"], r["best_bearing_deg"], r["confident"]))

print("\n== %d passed, %d failed ==" % (len(PASS), len(FAIL)))
sys.exit(1 if FAIL else 0)
