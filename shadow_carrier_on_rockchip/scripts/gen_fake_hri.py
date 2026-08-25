#!/usr/bin/env python3
"""fake_script.jsonl 生成器 - HRI 状态机自测剧本
场景: 跟随中主人停下打水 -> 拿着果冻缓慢走近 -> 快速空手走近
运行: python3 hri_state.py --fake fake_script.jsonl
"""
import json

def person(cx, cy, w, h, conf=0.8):
    return {"label": "person", "conf": conf,
            "bbox": [cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2]}

def obj(label, cx, cy, w, h, conf=0.7):
    return {"label": label, "conf": conf,
            "bbox": [cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2]}

frames = []
add = lambda wait, dets, moving=False: frames.append(
    {"wait": wait, "dets": dets, "moving": moving})

for i in range(10):
    add(0.5, [person(320 + (i % 3), 300, 120, 400)], moving=(i < 3))

for i in range(16):
    add(0.5, [person(320 + (i % 2), 300, 120, 400)])

for i in range(14):
    y = 200 + i * 12
    add(0.5, [person(320, y, 130, 420), obj("bottle", 320, y - 160, 40, 90)])

for i in range(10):
    add(0.5, [person(320, 380, 150, 450)])

for i in range(12):
    w, h = 130 + i * 25, 420 + i * 60
    add(0.5, [person(320, 470 - i * 20, w, h)])

for i in range(24):
    add(0.5, [person(320 + (i % 2), 470, 135, 420)])

for i in range(14):
    add(0.5, [person(320, 470, int(135 * (1.1 + i * 0.04)),
                     int(420 * (1.1 + i * 0.04)))])
for i in range(4):
    add(0.5, [person(320, 470, 189, 588)])
for i in range(12):
    b = max(0.68, 1.0 - (i + 1) * 0.09)
    add(0.5, [person(320, 470 + int(588 * (1 - b) / 2), 189, int(588 * b))])

with open("fake_script.jsonl", "w") as f:
    for fr in frames:
        f.write(json.dumps(fr) + "\n")
print(f"wrote {len(frames)} frames")
