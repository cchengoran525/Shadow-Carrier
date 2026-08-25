#!/usr/bin/env python3
"""c1_handheld_test.py - [HRI] C1 手持物检测置信度验证
对准摄像头手持目标物, 统计 /api/detections 中非person类的置信度分布
用法: python3 c1_handheld_test.py [时长秒] (默认30)
"""
import time, json, sys, urllib.request
from collections import defaultdict

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 30
API = "http://127.0.0.1:8080/api/detections"

stats = defaultdict(lambda: {"n": 0, "confs": [], "max": 0.0})
frames = hits = 0
t_end = time.time() + DURATION

while time.time() < t_end:
    try:
        with urllib.request.urlopen(API, timeout=1) as r:
            dets = json.load(r)
    except Exception:
        continue
    frames += 1
    hit = False
    for d in dets:
        label = d.get("label", "")
        if label == "person":
            continue
        conf = float(d.get("conf", 0))
        s = stats[label]
        s["n"] += 1
        s["confs"].append(conf)
        s["max"] = max(s["max"], conf)
        hit = True
    hits += hit
    time.sleep(0.2)

print(f"\n采样 {frames} 帧, 含物体帧 {hits} ({hits/max(1,frames)*100:.0f}%)")
print(f"{'类别':<14}{'帧数':>6}{'检出率':>8}{'平均conf':>10}{'最大conf':>10}")
for label, s in sorted(stats.items(), key=lambda kv: -kv[1]["max"]):
    mean = sum(s["confs"]) / len(s["confs"])
    rate = s["n"] / max(1, frames) * 100
    print(f"{label:<14}{s['n']:>6}{rate:>7.0f}%{mean:>10.2f}{s['max']:>10.2f}")

if not stats:
    print("无任何物体检出——检查物品摆放/距离/光照")
