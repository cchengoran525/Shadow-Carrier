#!/usr/bin/env python3
"""test_pick_safe_spot.py —— HIDE 落点回退顺序回归（离线，无需硬件）

10-22 机主指派修复：旧 `pick_safe_spot()` 的回退是"门优先"，直接把**门的方位**
当安全点 → `GOTO_SAFE <bearing>` 会朝门转身+前进 ≤1.5s，与 HIDE 语义相反
（README §HIDE 选点：门=交互亲和度极高→强惩罚；验收"靠墙且离门锥、不挡门"）。

顺带修：旧代码对 `map.free_sectors`（dict 列表）做 `abs(x)` → TypeError。
用法: python3 test_pick_safe_spot.py
"""
import json
import os
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hri_state

PASS = []
FAIL = []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    print("%-4s %s %s" % ("PASS" if cond else "FAIL", name, detail if not cond else ""))


TMP = tempfile.mkdtemp(prefix="hri_spot_")
GRID = os.path.join(TMP, "grid.json")


def write(grid):
    grid.setdefault("ts", time.time())
    with open(GRID, "w") as f:
        json.dump(grid, f, ensure_ascii=False)
    return GRID


# T1 打分 confident → 直接采用最佳方位
write({"ts": time.time(),
       "sector_scores": {"confident": True, "best_score": 1.7, "best_bearing_deg": -45.0}})
b, why = hri_state.pick_safe_spot(GRID)
check("T1 confident → 用打分方位", b == -45.0 and why.startswith("sector_score"), "%s %s" % (b, why))

# T2 打分不 confident + 只有门 → **背离门**（不能朝门走）
write({"ts": time.time(), "sector_scores": {"confident": False},
       "map": {"doors": [{"bearing_deg": 30, "dist_m": 3.0, "age_s": 1}], "free_sectors": []}})
b, why = hri_state.pick_safe_spot(GRID)
check("T2 只有门 → 背离门(-150°), 不朝门走", b == -150.0 and why.startswith("away_from_door"),
      "%s %s" % (b, why))

# T3 有自由方向 + 门 → 优先自由方向，且避开门的 ±45°
write({"ts": time.time(),
       "map": {"doors": [{"bearing_deg": 0, "dist_m": 4.0, "age_s": 1}], "free_sectors": []},
       "free_directions_deg": [0.0, 90.0, 180.0]})
b, why = hri_state.pick_safe_spot(GRID)
check("T3 自由优先且避开门口(0°) → 90°", b == 90.0 and why == "free", "%s %s" % (b, why))

# T4 map.free_sectors 是 dict 列表（旧代码 abs(dict) 会崩）
write({"ts": time.time(),
       "map": {"doors": [{"bearing_deg": 180, "dist_m": 4.0, "age_s": 1}],
               "free_sectors": [{"sector": 2, "ring": 0, "age_s": 1},
                                {"sector": 5, "ring": 0, "age_s": 1}]}})
b, why = hri_state.pick_safe_spot(GRID)
check("T4 dict 形式 free_sectors 不崩 → 90°(sector2)", b == 90.0, "%s %s" % (b, why))

# T5 过期 grid（>300s）→ None（HRI 走 goto_no_ref = 原地不动）
write({"ts": time.time() - 400,
       "sector_scores": {"confident": True, "best_score": 9.0, "best_bearing_deg": 0.0}})
b, why = hri_state.pick_safe_spot(GRID)
check("T5 过期 → None", b is None and "过期" in why, "%s %s" % (b, why))

# T6 文件缺失 → None
b, why = hri_state.pick_safe_spot(os.path.join(TMP, "nope.json"))
check("T6 缺失 → None", b is None and "不可读" in why, "%s %s" % (b, why))

# T7 自由方向全在门锥内 → 不得退回"朝门"; 应改走背离门
write({"ts": time.time(),
       "map": {"doors": [{"bearing_deg": 0, "dist_m": 3.0, "age_s": 1}], "free_sectors": []},
       "free_directions_deg": [0.0, 30.0]})
b, why = hri_state.pick_safe_spot(GRID)
check("T7 自由方向全在门锥 → 不朝门", b is not None and abs(((b + 180) % 360) - 180) > 45 and why.startswith("away_from_door"),
      "%s %s" % (b, why))

# T8 双门 0°/180° → 不能"背离 A = 朝 B"; 应选净空 ≥45° 的方向
write({"ts": time.time(), "sector_scores": {"confident": False},
       "map": {"doors": [{"bearing_deg": 0, "age_s": 1}, {"bearing_deg": 180, "age_s": 1}],
               "free_sectors": []}})
b, why = hri_state.pick_safe_spot(GRID)
check("T8 双门 → 不朝任一扇门", b is not None and min(abs(((b - d + 180) % 360) - 180) for d in (0, 180)) >= 45,
      "%s %s" % (b, why))

# T9 四门把四周占满(每 45° 都有门) → 无 ≥45° 净空 → 原地不动(None), 不赌一个方向
write({"ts": time.time(), "sector_scores": {"confident": False},
       "map": {"doors": [{"bearing_deg": d, "age_s": 1} for d in (0, 90, 180, 270)],
               "free_sectors": []}})
b, why = hri_state.pick_safe_spot(GRID)
check("T9 门环绕 → 原地不动", b is None and "无安全方向" in why, "%s %s" % (b, why))

print("\n== %d passed, %d failed ==" % (len(PASS), len(FAIL)))
sys.exit(1 if FAIL else 0)
