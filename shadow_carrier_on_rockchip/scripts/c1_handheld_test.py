#!/usr/bin/env python3
"""c1_handheld_test.py - [HRI] C1 手持物检测验证 (具名类别 + 类别无关关系判据)
对准摄像头手持目标物, 统计 /api/detections:
  ① 具名类别置信度分布 (bottle/cup/handbag/backpack/...)
  ② 类别无关"手中物"命中率: 任意非person框与主人bbox上半区重叠即算
用法: python3 c1_handheld_test.py [时长秒] (默认30)
"""
import time, json, sys, urllib.request
from collections import defaultdict

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 30
API = "http://127.0.0.1:8080/api/detections"
HOLD_ANY_TOP_FRAC = 0.6
HOLD_ANY_MIN_CONF = 0.4
HOLD_IOU_MIN = 0.05


def iou_ratio(inner, outer):
    ix1, iy1 = max(inner[0], outer[0]), max(inner[1], outer[1])
    ix2, iy2 = min(inner[2], outer[2]), min(inner[3], outer[3])
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    a_in = max(1e-6, (inner[2] - inner[0]) * (inner[3] - inner[1]))
    return (iw * ih) / a_in


stats = defaultdict(lambda: {"n": 0, "confs": [], "max": 0.0})
frames = frames_person = frames_inhand = 0
inhand_classes = defaultdict(int)
t_end = time.time() + DURATION

while time.time() < t_end:
    try:
        with urllib.request.urlopen(API, timeout=1) as r:
            dets = json.load(r).get("detections", [])
    except Exception:
        continue
    frames += 1
    persons = [d for d in dets if d.get("c") == "person" and d.get("p", 0) >= 0.5]
    objs = [d for d in dets if d.get("c") != "person"]
    if persons:
        frames_person += 1
    hit = False
    for d in objs:
        label, conf = d.get("c", ""), float(d.get("p", 0))
        s = stats[label]
        s["n"] += 1
        s["confs"].append(conf)
        s["max"] = max(s["max"], conf)
        if persons and conf >= HOLD_ANY_MIN_CONF:
            p = max(persons, key=lambda x: x.get("p", 0))
            upper = [p["x1"], p["y1"], p["x2"], p["y1"] + (p["y2"] - p["y1"]) * HOLD_ANY_TOP_FRAC]
            if iou_ratio([d["x1"], d["y1"], d["x2"], d["y2"]], upper) >= HOLD_IOU_MIN:
                hit = True
                inhand_classes[label] += 1
    frames_inhand += hit
    time.sleep(0.2)

print(f"\n采样 {frames} 帧 | 含主人 {frames_person} | **手中物命中 {frames_inhand} "
      f"({frames_inhand/max(1,frames_person)*100:.0f}% of 含主人帧)**")
print(f"\n{'类别':<14}{'帧数':>6}{'检出率':>8}{'平均conf':>10}{'最大conf':>10}")
for label, s in sorted(stats.items(), key=lambda kv: -kv[1]["max"]):
    mean = sum(s["confs"]) / len(s["confs"])
    print(f"{label:<14}{s['n']:>6}{s['n']/max(1,frames)*100:>7.0f}%{mean:>10.2f}{s['max']:>10.2f}")

if inhand_classes:
    print("\n被判为'手中物'的类别:", dict(inhand_classes))
else:
    print("\n未命中任何'手中物'——检查物品是否在主人上半身框内/距离/光照")
