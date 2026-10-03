#!/usr/bin/env python3
"""fake_script.jsonl 生成器 - HRI 状态机自测剧本 (物理一致版)
用 hri_params.json 的 K=fx*person_h 反推每个距离应有的 bbox 高度,
场景按"真实距离序列"编排, 与 metric 模式同源。
场景: 跟随→静止(打水)→持瓶缓近→后退→空手快冲→长静止(找安全点)→走近弯腰递物→保持弯腰
运行: python3 gen_fake_hri.py && python3 hri_state.py --fake fake_script.jsonl
"""
import json, os

HERE = os.path.dirname(os.path.abspath(__file__))
P = json.load(open(os.path.join(HERE, "hri_params.json")))
CAM = P["camera"]
K = CAM["focal_px"] * CAM["person_height_m"]
KW = CAM["focal_px"] * CAM["shoulder_width_m"]
FRAME_H = CAM["frame_h"]
FRAME_W = 640
CX = 320


def person(dist, jitter=0.0, shrink=1.0):
    h_full = min(FRAME_H - 10, K / dist)
    h = h_full * shrink
    w = min(FRAME_W - 10, KW / dist)
    cx = CX + jitter
    return {"label": "person", "conf": 0.8,
            "bbox": [cx - w / 2, FRAME_H - 5 - h, cx + w / 2, FRAME_H - 5]}


def obj(label, dist, shrink=1.0, conf=0.7):
    h = min(FRAME_H - 10, K / dist) * shrink
    w = h * 0.35
    top = FRAME_H - 5 - h
    return {"label": label, "conf": conf,
            "bbox": [CX - w / 4, top + h * 0.15, CX + w / 4, top + h * 0.15 + h * 0.4]}


frames = []
add = lambda wait, dets, moving=False: frames.append(
    {"wait": wait, "dets": dets, "moving": moving})


def ramp(d0, d1, n, dets_fn, wait=0.5):
    for i in range(n):
        d = d0 + (d1 - d0) * i / max(1, n - 1)
        add(wait, dets_fn(d))


for i in range(8):
    add(0.5, [person(2.6, jitter=(i % 3) - 1)], moving=(i < 3))

for i in range(16):
    add(0.5, [person(2.6, jitter=(i % 2) - 0.5)])

ramp(2.6, 1.3, 14, lambda d: [person(d), obj("bottle", d, shrink=0.35)])

ramp(1.3, 1.8, 6, lambda d: [person(d)])

ramp(2.6, 0.7, 5, lambda d: [person(d)])
for i in range(4):
    add(0.5, [person(0.7)])

for i in range(48):
    add(0.5, [person(2.6, jitter=(i % 2) - 0.5)])

ramp(2.6, 1.0, 8, lambda d: [person(d)])
for i in range(4):
    add(0.5, [person(1.0)])

for i in range(12):
    shrink = max(0.68, 1.0 - (i + 1) * 0.09)
    add(0.5, [person(1.0, shrink=shrink)])

for i in range(12):
    add(0.5, [person(1.0, shrink=0.68)])

with open(os.path.join(HERE, "fake_script.jsonl"), "w") as f:
    for fr in frames:
        f.write(json.dumps(fr) + "\n")
print(f"wrote {len(frames)} frames, K={K:.0f}px*m")
