#!/usr/bin/env python3
"""test_stamp_objects.py —— ring=3(超远)物体落格回归（2026-10-24 白名单#4）

背景: polar_to_sector(任意角, dist>3.0m) 返回 ring=3, 而 grid 只有 3 行。
修复前 capture 内联 `grid[o["ring"]][o["sector"]]` 对 3m 外 YOLO 目标直接
IndexError —— once/scan 整次快照失败, loop 每轮 cycle error, HIDE 断粮。
修复后抽出 stamp_objects() 并对越界 ring/sector 跳过。

用法: python3 test_stamp_objects.py   (依赖 cv2/numpy —— geometry_daemon 顶层 import)
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import polar_mapper as pm                                  # noqa: E402
import geometry_daemon as gd                               # noqa: E402


def _grid(rows=3, cols=8):
    return [[{"label": "free", "sources": []} for _ in range(cols)] for _ in range(rows)]


def main():
    # 1) 语义锚: polar_to_sector 对 >3m 返回 ring == len(RINGS_M) == 3(无格可落)
    s, r = pm.polar_to_sector(10.0, 3.5)
    assert r == 3 and r == len(pm.RINGS_M), f">3m 应返回超远 ring=3, 实际 {r}"
    print(f"  polar_to_sector(10°, 3.5m) -> sector={s} ring={r}(超远) ✓")

    # 2) 修复点: 超远物体 + 正常物体混合落格, 不再 IndexError
    g = _grid()
    objs = [
        {"cls": "person", "ring": 3, "sector": 0, "bearing_deg": 5.0, "dist_m": 4.2},   # 超远(旧代码崩)
        {"cls": "person", "ring": 1, "sector": 2, "bearing_deg": 100.0, "dist_m": 1.8},  # 正常
    ]
    doors = [{"cls": "door", "ring": 0, "sector": 7, "bearing_deg": -170.0, "dist_m": 2.9}]
    gd.stamp_objects(g, objs, doors)   # 修复前: IndexError: list index out of range
    assert g[1][2]["label"] == "obj:person", "正常物体必须落格"
    assert g[0][7]["label"] == "obj:door", "门必须落格"
    assert g[0][0]["label"] == "free", "超远(ring=3)物体不得挤占任何格子"
    assert g[2][0]["label"] == "free"
    print("  超远跳过 · 正常/门落格 ✓")

    # 3) 边角: 越界 sector 同样跳过(防御, 当前 polar_to_sector 不会产出)
    g2 = _grid()
    gd.stamp_objects(g2, [{"cls": "box", "ring": 2, "sector": 8,
                           "bearing_deg": 0.0, "dist_m": 2.0}], [])
    assert all(c["label"] == "free" for row in g2 for c in row)
    print("  越界 sector 防御性跳过 ✓")

    print("\n全部断言通过")


if __name__ == "__main__":
    main()
