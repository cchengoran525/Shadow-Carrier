#!/usr/bin/env python3
"""test_geometry_map_wrap.py —— 方位角 EWMA 跨±180缝回归（2026-10-24 白名单#5）

背景: RollingMap 对门/物体的 bearing_deg 做线性 EWMA。172° 与 -172° 是同一
物理方向(正后方), 旧公式 (1-0.4)*172 + 0.4*(-172) = 34.4° —— 门瞬移到右前方,
污染 sector_score 选点与 hri_state 的 HIDE 判据。
修复: _ewma_bearing() 按最短角差更新。

用法: python3 test_geometry_map_wrap.py   (纯 stdlib, 无 cv2 依赖)
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from geometry_map import RollingMap, _ewma_bearing, EWMA   # noqa: E402


def _snap(bearing, cls="door", dist=2.5):
    return {"ts": 0, "pose": {}, "objects": [
        {"cls": cls, "bearing_deg": bearing, "dist_m": dist}]}


def main():
    # 1) 单元: 跨缝单次更新 —— 172° 收到 -172° 观测, 应停在 ±180 附近而不是 34°
    b = _ewma_bearing(172.0, -172.0)
    assert abs(b) >= 170.0, f"172/-172 跨缝应停在 ±180 附近, 实际 {b}"
    print(f"  _ewma_bearing(172, -172) = {b}  (旧公式会得 34.4) ✓")

    # 2) 单元: 非跨缝与旧公式逐值一致(EWMA={ewma})
    assert _ewma_bearing(10.0, 20.0) == round(10.0 + EWMA * 10.0, 1)
    assert _ewma_bearing(-30.0, -10.0) == round(-30.0 + EWMA * 20.0, 1)
    print("  非跨缝与旧公式逐值一致 ✓")

    # 3) 集成: 同一扇门先 172° 后 -172° 两次观测, 门不得瞬移到右前方
    m = RollingMap()
    m.update(_snap(172.0))
    m.update(_snap(-172.0))
    assert len(m.doors) == 1, "方位差 16°(跨缝)应关联为同一扇门"
    bd = m.doors[0]["bearing_deg"]
    assert abs(bd) >= 170.0, f"门方位应仍在正后方(|b|≥170), 实际 {bd}"
    assert m.doors[0]["sector"] in (3, 4), f"正后方应落扇区 3/4, 实际 {m.doors[0]['sector']}"
    print(f"  门 172°→-172° 两次更新: bearing={bd} sector={m.doors[0]['sector']} ✓")

    # 4) 集成: 物体(非门)同样不跨缝瞬移
    m2 = RollingMap()
    m2.update(_snap(178.0, cls="fridge"))
    m2.update(_snap(-178.0, cls="fridge"))
    assert len(m2.objects) == 1
    assert abs(m2.objects[0]["bearing_deg"]) >= 175.0
    print(f"  冰箱 178°→-178°: bearing={m2.objects[0]['bearing_deg']} ✓")

    print("\n全部断言通过")


if __name__ == "__main__":
    main()
