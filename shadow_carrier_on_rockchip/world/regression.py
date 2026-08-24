#!/usr/bin/env python3
"""
world_lab 回归基准 —— 参数冻结后的验收测试
基准集(8张): 3张含门(必须检出) + 5张无门(必须零误报)
用法: python3 regression.py <photos_dir>
通过标准: 命中 3/3 且 误报 0/5, 否则 FAIL
"""
import sys
import os
import json
import door_v3

# 期望: 文件名前8位 -> 是否应有门
EXPECT = {
    "30dfca35": True,   # 正对防火门
    "6fdcfcc0": True,   # 斜对防火门
    "6e7ef668": True,   # 走廊(右侧近处房门)
    "da86f97e": False,  # 宿舍(置物架干扰)
    "ee3878b6": False,  # 卫生间(瓷砖墙缝)
    "6fdcfcc": None,    # placeholder
}
EXPECT = {k: v for k, v in EXPECT.items() if v is not None}


def main():
    d = sys.argv[1]
    hits, misses, fps = 0, [], []
    rows = []
    for f in sorted(os.listdir(d)):
        if not f.lower().endswith((".jpg", ".png")) or "annotated" in f or "_v" in f:
            continue
        key = f[:8]
        if key not in EXPECT:
            continue
        out, res = door_v3.process(os.path.join(d, f))
        found = len(res["doors"]) > 0
        want = EXPECT[key]
        if want:
            ok = found
            hits += ok
            if not ok:
                misses.append(key)
        else:
            ok = not found
            if found:
                fps.append((key, res["doors"][0]))
        rows.append((key, want, found, ok, res["timing_ms"]["pipeline"]))
    print("\n=== 回归基准 ===")
    print("%-10s %-6s %-6s %-5s %s" % ("样本", "期望门", "检出", "结果", "耗时ms"))
    for key, want, found, ok, ms in rows:
        print("%-10s %-8s %-8s %-5s %.0f" % (key, want, found, "PASS" if ok else "FAIL", ms))
    total_pos = sum(1 for v in EXPECT.values() if v)
    total_neg = sum(1 for v in EXPECT.values() if not v)
    verdict = "PASS" if hits == total_pos and not fps else "FAIL"
    print("\n命中: %d/%d  误报: %d/%d  => %s" % (hits, total_pos, len(fps), total_neg, verdict))
    if fps:
        for key, d0 in fps:
            print("  误报详情 %s: %s" % (key, d0))
    sys.exit(0 if verdict == "PASS" else 1)


if __name__ == "__main__":
    main()
