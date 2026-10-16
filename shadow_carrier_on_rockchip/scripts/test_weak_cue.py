#!/usr/bin/env python3
"""test_weak_cue.py — 底盘转向弱线索 (owner_id.weak_cue) 离线自测, 无需硬件

覆盖:
  ① low_conf 源: 被层0拦下的低置信/贴底框 → 给出方向 (层0已收的框不重复消费)
  ② leg_band 源: 认主派生的下装模板 + 画面下半带颜色匹配 (擦身只剩腿的工况)
  ③ 护栏: 弱线索不产生主人候选 (不写 owner.json / 不进 select_target)
用法: python3 test_weak_cue.py      (断言失败退出非0)
"""
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import owner_id  # noqa: E402

print("== ① low_conf 源 ==")
# 右侧、贴底(person 框被画面底边裁切 → 层0会拦, 弱通道该收)
d = [{"c": "person", "p": 0.35, "x1": 500, "y1": 200, "x2": 600, "y2": 479}]
c = owner_id.weak_cue(profile=None, dets=d)
print(f"  右侧贴底低置信框 → {c}")
assert c and c["src"] == "low_conf" and c["dir"] == "R", c
d = [{"c": "person", "p": 0.40, "x1": 40, "y1": 200, "x2": 140, "y2": 479}]
c = owner_id.weak_cue(profile=None, dets=d)
assert c and c["dir"] == "L", c
# 层0会收的框(高置信 + 高度够) → 不走弱通道(避免与强通道重复)
d = [{"c": "person", "p": 0.90, "x1": 500, "y1": 200, "x2": 600, "y2": 479}]
assert owner_id.weak_cue(profile=None, dets=d) is None
# 置信度过低 → 连弱线索都不算(幻影兜底)
d = [{"c": "person", "p": 0.10, "x1": 500, "y1": 200, "x2": 600, "y2": 479}]
assert owner_id.weak_cue(profile=None, dets=d) is None
# 非 person 类别永不进入
d = [{"c": "bottle", "p": 0.45, "x1": 500, "y1": 200, "x2": 600, "y2": 479}]
assert owner_id.weak_cue(profile=None, dets=d) is None
print("  [PASS] 低置信/贴底框给出方向; 层0已收/过低置信/非person 全部拒绝")

print("\n== ② leg_band 源 (擦身只剩腿) ==")
img = np.full((480, 640, 3), 40, np.uint8)
img[300:470, 120:200] = (30, 200, 220)          # BGR 亮橙"下装", 在画面左侧
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
leg = owner_id._roi_hist(hsv, {"x1": 120, "y1": 300, "x2": 200, "y2": 470})
assert leg is not None
prof = owner_id.OwnerProfile(leg, 0.4, 10, 120.0, leg)
c = owner_id.weak_cue(profile=prof, dets=[], img_bgr=img)
print(f"  下半带亮色块 → {c}")
assert c and c["src"] == "leg_band" and c["dir"] == "L", c
assert 0.0 < c["score"] <= 1.0, c
# 换到右侧 → 方向反过来
img2 = np.full((480, 640, 3), 40, np.uint8)
img2[300:470, 440:520] = (30, 200, 220)
c = owner_id.weak_cue(profile=prof, dets=[], img_bgr=img2)
assert c and c["src"] == "leg_band" and c["dir"] == "R", c
# 画面里没有该颜色 → 无线索(不乱指)
c = owner_id.weak_cue(profile=prof, dets=[], img_bgr=np.full((480, 640, 3), 40, np.uint8))
assert c is None, c
# 没认主(无下装模板) → 无 leg_band 源
no_leg = owner_id.OwnerProfile(leg, 0.4, 10, 120.0, None)
assert owner_id.weak_cue(profile=no_leg, dets=[], img_bgr=img) is None
print("  [PASS] 腿带匹配给方向; 无该颜色/无模板时不出线索")

print("\n== ③ 护栏: 弱线索不产生主人候选 ==")
# 只有低置信框时, 强通道(select_target 的入口)拿不到任何候选
assert owner_id._fetch_detections.__doc__ and \
    "第0层" in owner_id._fetch_detections.__doc__
src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "owner_id.py"), encoding="utf-8").read()
for bad in ("weak_cue", "find_leg_band"):
    seg = src.split(f"def {bad}")[1].split("\ndef ")[0]
    parts = seg.split('"""')                    # 去掉 docstring: 里面是"绝不写…"的说明
    body = "".join(parts[0::2]) if len(parts) > 1 else seg
    body = "\n".join(l for l in body.splitlines() if not l.strip().startswith("#"))
    for forbidden in ("owner.json", "_publish", "select_target(", "tracker."):
        assert forbidden not in body, f"{bad} 内出现 {forbidden} → 违反第0层"
print("  [PASS] 弱线索函数体内无 owner.json/候选池/跟踪器 写入")

print("\n全部断言通过")
