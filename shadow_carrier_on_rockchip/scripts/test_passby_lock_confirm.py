#!/usr/bin/env python3
"""test_passby_lock_confirm.py — 擦身防吸路人(换人确认门)离线回归, 无需硬件

场景: 主人连续跟随后擦身出画; 同色路人(单帧闪 / 持续 / 出画侧反方向)试图顶替锁定。
期望: 单帧闪不生效; 反向候选需 5 帧; 同向候选需 3 帧; 待确认期间不发布 owner.json。
用法: python3 test_passby_lock_confirm.py   (断言失败退出非0)
"""
import importlib.util
import os
import sys
import time
import types

import numpy as np

SCRIPTS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPTS)
spec = importlib.util.spec_from_file_location("follow_controller", os.path.join(SCRIPTS, "follow_controller.py"))
fc_mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fc_mod)
import owner_id  # noqa: E402

IMG = np.zeros((480, 640, 3), np.uint8)
PUBLISHED = []
fc_mod._publish_owner = lambda box: PUBLISHED.append(dict(box))

# 队列式 select_target 替身: ("box"|None, score)
QUEUE = []
owner_id._fetch_detections = lambda: [{"c": "person", "x1": 1, "y1": 1, "x2": 2, "y2": 2, "p": 0.9}]
owner_id.read_recent_frame = lambda: IMG


def _stub_select(profile, img, dets, tracker=None):
    if not QUEUE:
        return None, 0.0
    box, score = QUEUE.pop(0)
    return box, score


owner_id.select_target = _stub_select

CMDS = []
fc = fc_mod.FollowController(lambda c: CMDS.append(c))
fc.running = True
fc.profile = object()          # 非None → 走真实认主路径

def bx(cx, cy=240, h=300):
    return {"x1": cx - 40, "y1": cy - h / 2, "x2": cx + 40, "y2": cy + h / 2}

def tick():
    fc._last_t = time.monotonic() - 0.1
    _t, _v, info = fc.tick()
    return fc._fetch_cache if hasattr(fc, "_fetch_cache") else None, info

def feed(box, score, n=1):
    """排入 n 帧同一定位, 逐拍执行, 返回每拍是否被接受(person is not None)"""
    QUEUE.clear()
    for _ in range(n):
        QUEUE.append((box, score))
    outs = []
    for _ in range(n):
        fc._last_t = time.monotonic() - 0.1
        p = fc._fetch_person()          # 直接测选人门, 隔离 tick 其余逻辑
        outs.append(p is not None)
        fc.lost = 0 if p is not None else fc.lost + 1   # 忠实:tick 成功即清零丢失
    return outs

print("== A. 首次锁定不延迟 ==")
r = feed(bx(320), 0.7, 1)
assert r == [True], r
print("  [PASS] 认主后首帧即接受(无确认延迟)")

print("== B. 连续跟踪不延迟 ==")
r = feed(bx(360), 0.7, 3)
assert r == [True, True, True], r
print("  [PASS] 连续位移每帧直接接受")

print("== C. 主人出画(1拍空) ==")
r = feed(None, 0.0, 1)
assert r == [False], r
fc.lost = 1                         # 忠实时序: tick 会累加丢失计数
fc._exit_sign = "R"                 # 正常由 _steer 在贴边时置位
print("  [PASS] 空帧不进候选")

print("== D. 反向路人(出画侧R, 路人在L) → 需 5 帧 ==")
r = feed(bx(120), 0.60, 6)
assert r[:4] == [False, False, False, False], r
assert r[4] is True, r
print(f"  [PASS] 前4帧拒绝、第5帧确认: {r}")

print("== E. 同向路人(路人在R, 与出画侧同侧) → 需 3 帧 ==")
fc2 = fc_mod.FollowController(lambda c: None)
fc2.running = True; fc2.profile = object()
QUEUE.clear(); QUEUE.append((bx(320), 0.7)); fc2._last_t = time.monotonic() - 0.1
assert fc2._fetch_person() is not None          # 首次锁定
fc2._exit_sign = "R"
QUEUE.clear(); QUEUE.append((None, 0.0)); fc2._last_t = time.monotonic() - 0.1
fc2._fetch_person(); fc2.lost = 1
for i in range(1, 5):
    QUEUE.clear(); QUEUE.append((bx(560), 0.60)); fc2._last_t = time.monotonic() - 0.1
    got = fc2._fetch_person() is not None
    fc2.lost = 0 if got else fc2.lost + 1
    print(f"    第{i}帧: {'接受' if got else '拒绝'}")
    if i < 3:
        assert not got
    elif i == 3:
        assert got
print("  [PASS] 同向候选第3帧确认")

print("== F. 单帧闪: 路人1帧后主人回来 ==")
fc3 = fc_mod.FollowController(lambda c: None)
fc3.running = True; fc3.profile = object()
PUBLISHED.clear()
QUEUE.clear(); QUEUE.append((bx(300), 0.7)); fc3._last_t = time.monotonic() - 0.1
assert fc3._fetch_person() is not None
QUEUE.clear(); QUEUE.append((None, 0.0)); fc3._last_t = time.monotonic() - 0.1
fc3._fetch_person(); fc3.lost = 1
QUEUE.clear(); QUEUE.append((bx(120), 0.60)); fc3._last_t = time.monotonic() - 0.1
assert fc3._fetch_person() is None                 # 路人单帧: 待确认
fc3.lost += 1
assert len(PUBLISHED) == 1                         # 只发布过首次主人框
QUEUE.clear()
for _ in range(5):
    QUEUE.append((bx(300), 0.7))
res = []
for _ in range(5):
    fc3._last_t = time.monotonic() - 0.1
    _ok = fc3._fetch_person() is not None
    fc3.lost = 0 if _ok else fc3.lost + 1
    res.append(_ok)
assert True in res, res
assert len(PUBLISHED) == 1 + res.count(True), (len(PUBLISHED), res)
print(f"  [PASS] 路人未发布 owner.json; 主人回来后确认链重建 {res}")

print("== G. 分数不过线(SCORE_MIN+margin) → 永不确认 ==")
fc4 = fc_mod.FollowController(lambda c: None)
fc4.running = True; fc4.profile = object()
QUEUE.clear(); QUEUE.append((bx(320), 0.7)); fc4._last_t = time.monotonic() - 0.1
fc4._fetch_person()
fc4._exit_sign = "R"
QUEUE.clear(); QUEUE.append((None, 0.0)); fc4._last_t = time.monotonic() - 0.1
fc4._fetch_person(); fc4.lost = 1
r = feed.__wrapped__ if hasattr(feed, "__wrapped__") else None
outs = []
for _ in range(10):
    QUEUE.clear(); QUEUE.append((bx(120), owner_id.SCORE_MIN)); fc4._last_t = time.monotonic() - 0.1
    _ok = fc4._fetch_person() is not None
    fc4.lost = 0 if _ok else fc4.lost + 1
    outs.append(_ok)
assert not any(outs), outs
print("  [PASS] 低分候选即使持久也不确认")

print("\n全部断言通过")
