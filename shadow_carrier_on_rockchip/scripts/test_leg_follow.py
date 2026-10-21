#!/usr/bin/env python3
"""test_leg_follow.py — 腿跟(LEG_FOLLOW_ENABLED) 离线门控测试 (无需硬件)

覆盖 2026-10-22 统筹裁定 A 的控制器侧门:
  TTL / 方向±25° / 跳变120px / 腿带丢失0.4s / 强锁恢复 / 速度包络 120–140.
外部硬门(mode/HRI/1人/OBS/dist/score)在 rk_control._leg_follow_cue, 本测试用注入 cue 代替.
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import follow_controller as m

m.TURN_ENABLED = False
m.LEG_FOLLOW_ENABLED = True


def make(cue_fn, person=None):
    cmds = []
    c = m.FollowController(
        cmds.append,
        person_provider=lambda: person,
        yaw_rate_fn=lambda: 0.0,
        bearing_of_fn=lambda x: math.degrees(math.atan((x - 320.0) / 508.2)),
        leg_follow_fn=cue_fn)
    c.running = True
    c.profile = "test"
    c.scx = 320.0
    c.sh = 300.0
    c._last_strong_t = time.monotonic()
    return c, cmds


def last_diff(cmds):
    ds = [x for x in cmds if x.startswith("DIFF")]
    assert ds, f"no DIFF: {cmds}"
    d = ds[-1]
    return int(d.split("L")[1].split("R")[0]), int(d.split("R")[1]), d


def main():
    # 1) 腿带正前 → leg_follow，速度 120–140
    c, cmds = make(lambda: {"u": 320, "score": 0.6, "src": "leg_band"})
    tag, val, info = c.tick()
    assert tag == "DIFF" and info.get("state") == "leg_follow", (tag, val, info)
    l, r, d = last_diff(cmds)
    assert 120 <= min(l, r) and max(l, r) <= 140, d
    print(f"[PASS] 正前腿带 → leg_follow  {d} state={info['state']}")

    # 2) 人偏右 → 左轮快(右转)
    c, cmds = make(lambda: {"u": 420, "score": 0.6, "src": "leg_band"})
    tag, val, info = c.tick()
    l, r, d = last_diff(cmds)
    assert l > r, d
    print(f"[PASS] 腿带偏右 → 右转  {d}")

    # 3) 方向门 ±25°: u=560 → 25.3° → STOP/wait
    c, cmds = make(lambda: {"u": 560, "score": 0.6, "src": "leg_band"})
    tag, val, info = c.tick()
    assert tag == "STOP" and info.get("state") == "wait_owner" and info.get("why") == "leg_gate", (tag, val, info)
    print(f"[PASS] 方向超 ±25° → STOP/wait  {info}")

    # 4) TTL: 强锁 4s 前 → 不走腿跟
    c, cmds = make(lambda: {"u": 320, "score": 0.6, "src": "leg_band"})
    c._last_strong_t = time.monotonic() - 4.0
    tag, val, info = c.tick()
    assert info.get("state") != "leg_follow", (tag, val, info)
    print(f"[PASS] TTL 过期 → 不走腿跟  state={info.get('state')}")

    # 5) 跳变门: 300 → 500 (>120px)
    c, cmds = make(lambda: {"u": 300, "score": 0.6, "src": "leg_band"})
    tag, val, info = c.tick()
    assert info.get("state") == "leg_follow", (tag, val, info)
    c.leg_follow_fn = lambda: {"u": 500, "score": 0.6, "src": "leg_band"}
    tag, val, info = c.tick()
    assert tag == "STOP" and info.get("why") == "leg_gate", (tag, val, info)
    print(f"[PASS] 腿带跳变 >120px → STOP/wait  {info}")

    # 6) 腿带丢失 <0.4s → hold; >0.4s → STOP
    c, cmds = make(lambda: {"u": 320, "score": 0.6, "src": "leg_band"})
    tag, val, info = c.tick()
    assert info.get("state") == "leg_follow"
    c.leg_follow_fn = lambda: None
    tag, val, info = c.tick()
    assert info.get("state") == "leg_hold", (tag, val, info)
    c._leg_last_t = time.monotonic() - 1.0
    tag, val, info = c.tick()
    assert tag == "STOP" and info.get("why") == "leg_lost", (tag, val, info)
    print("[PASS] 腿带丢失 <0.4s hold → >0.4s STOP/wait")

    # 7) 有强 person → 正常路径, 不是 leg_follow
    c, cmds = make(lambda: {"u": 320, "score": 0.6, "src": "leg_band"},
                   person={"cx": 320, "h": 300, "cy": 240})
    tag, val, info = c.tick()
    assert info.get("state") != "leg_follow", (tag, val, info)
    print(f"[PASS] 强锁存在 → 正常跟随  state={info.get('state')}")

    # 8) 累计里程上限 0.6m → STOP/wait (不只看时间)
    c, cmds = make(lambda: {"u": 320, "score": 0.6, "src": "leg_band"})
    hit = None
    for i in range(60):
        c._last_t = time.monotonic() - 0.1     # 模拟 10Hz
        c._leg_travel_t = time.monotonic() - 0.1   # 里程死推同样按 10Hz
        tag, val, info = c.tick()
        if info.get("why") == "leg_travel_cap":
            hit = i
            break
    assert hit is not None and tag == "STOP", (hit, tag, info)
    print(f"[PASS] 累计里程 ≥0.6m → STOP/wait (第{hit}拍, why=leg_travel_cap)")

    print("\nALL PASS (8/8)")


if __name__ == "__main__":
    main()
