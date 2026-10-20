#!/usr/bin/env python3
"""test_hri_bridge.py — A1.3 HRI 动作桥离线回归 (2026-10-20)

覆盖 `rk_control` 新增的 HRI 动作消费端:
  A. `_hri_action`   新鲜/过期/坏 JSON/无 action
  B. `_hri_plan`     APPROACH 近距/障碍/间隙; BACK_OFF 间隙; GOTO_SAFE 闩锁/状态切换
  C. `_hri_execute`  有界脉冲、朝向符号(右正→MOVE R)、IMU 丢失不动、速度上限

全部注入(不发串口/不读 /dev/shm):
用法: python3 test_hri_bridge.py
"""
import json
import os
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rk_control as rk  # noqa: E402

tmp = tempfile.mkdtemp()
sent = []


def send(cmd):
    sent.append(cmd)
    return True


class Clock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t

    def sleep(self, s):
        self.t += s


def write(name, payload):
    p = os.path.join(tmp, name)
    with open(p, "w", encoding="utf-8") as f:
        json.dump(payload, f)
    return p


print("== A. _hri_action: 新鲜 / 过期 / 无 action / 坏 JSON ==")
p_ok = write("ok.json", {"ts": time.time(), "state": "RECEIVE", "action": "APPROACH"})
assert rk._hri_action(path=p_ok)["action"] == "APPROACH"
p_old = write("old.json", {"ts": time.time() - 5, "state": "RECEIVE", "action": "APPROACH"})
assert rk._hri_action(path=p_old) is None
p_none = write("none.json", {"ts": time.time(), "state": "FOLLOW", "action": None})
assert rk._hri_action(path=p_none) is None
p_bad = os.path.join(tmp, "bad.json")
open(p_bad, "w").write("{oops")
assert rk._hri_action(path=p_bad) is None
print("  新鲜→动作 · 过期→None · 无动作→None · 坏JSON→None")

print("\n== B. _hri_plan: 近距 / 障碍 / 间隙 / 闩锁 ==")
rk._hri_reset()
clk = Clock()
o_app = {"state": "RECEIVE", "action": "APPROACH"}
assert rk._hri_plan(o_app, clk.now(), dist=1.0) == ("approach", None)
assert rk._hri_plan(o_app, clk.now(), dist=1.0) is None            # 间隙内
clk.sleep(2.0)
assert rk._hri_plan(o_app, clk.now(), dist=0.4) is None            # 已够近
assert rk._hri_plan(o_app, clk.now(), dist=1.0, obs_cm=20) is None  # 超声太近
assert rk._hri_plan(o_app, clk.now(), dist=1.0, obs_cm=80) == ("approach", None)
print("  首次→脉冲 · 间隙内→None · <0.5m→None · 超声<35cm→None")

rk._hri_reset()
clk = Clock()
o_back = {"state": "YIELD", "action": "BACK_OFF"}
assert rk._hri_plan(o_back, clk.now()) == ("back", None)
assert rk._hri_plan(o_back, clk.now()) is None
clk.sleep(2.0)
assert rk._hri_plan(o_back, clk.now()) == ("back", None)
print("  BACK_OFF: 首次→脉冲 · 间隙内→None · 2s 后可再来")

rk._hri_reset()
clk = Clock()
o_goto = {"state": "HIDE", "action": "GOTO_SAFE 45"}
assert rk._hri_plan(o_goto, clk.now()) == ("goto", 45.0)
assert rk._hri_plan(o_goto, clk.now() + 3.0) is None                    # 闩锁
assert rk._hri_plan({"state": "WAIT", "action": "GOTO_SAFE 45"},        # 状态切换
                    clk.now() + 3.0) == ("goto", 45.0)
rk._hri_plan({"state": "WAIT", "action": None}, clk.now() + 3.0)        # 清闩锁
assert rk._hri_plan(o_goto, clk.now() + 3.0) == ("goto", 45.0)
print("  GOTO_SAFE: 首次→goto · 闩锁→None · 状态/动作变→可重来")

print("\n== C. _hri_execute: 有界脉冲 + 朝向符号 + 安全 ==")
rk._hri_reset()
clk = Clock()
sent.clear()
seq = [1.0, 0.9, 0.4]


def dist():
    return seq.pop(0) if seq else 0.4


n = rk._hri_execute(("approach", None), send=send, dist_fn=dist,
                    obs_fn=lambda: (False, 80), sleep_fn=clk.sleep, now_fn=clk.now)
assert n == "approach" and sent[-1] == "STOP", sent
assert all(c.startswith("MOVE F") for c in sent[:-1]), sent
assert all(int(c.split()[2]) <= 120 for c in sent if c.startswith("MOVE F")), sent
print(f"  APPROACH: {len(sent) - 1} 个前进脉冲 → STOP")

clk = Clock()
sent.clear()
rk._hri_execute(("approach", None), send=send, dist_fn=lambda: 1.0,
                obs_fn=lambda: (True, 20), sleep_fn=clk.sleep, now_fn=clk.now)
assert sent == ["STOP"], sent
print("  障碍 → 立即 STOP（不前进）")

clk = Clock()
sent.clear()
rk._hri_execute(("back", None), send=send, sleep_fn=clk.sleep, now_fn=clk.now)
assert sent[0] == f"MOVE B {rk.HRI_BACK_SPD}" and sent[-1] == "STOP", sent
print(f"  BACK_OFF: {sent}")

clk = Clock()
sent.clear()
yaws = [0.0, -10.0, -40.0, -45.0, -45.0]


def yaw_r():
    return yaws.pop(0) if yaws else -45.0


rk._hri_execute(("goto", 45.0), send=send, yaw_fn=yaw_r,
                obs_fn=lambda: (False, 80), sleep_fn=clk.sleep, now_fn=clk.now)
assert any(c.startswith("MOVE R") for c in sent), sent
assert sent[-1] == "STOP", sent
print("  GOTO_SAFE +45°(右正) → MOVE R ... → STOP")

clk = Clock()
sent.clear()
yaws = [0.0, 10.0, 40.0, 45.0, 45.0]


def yaw_l():
    return yaws.pop(0) if yaws else 45.0


rk._hri_execute(("goto", -45.0), send=send, yaw_fn=yaw_l,
                obs_fn=lambda: (False, 80), sleep_fn=clk.sleep, now_fn=clk.now)
assert any(c.startswith("MOVE L") for c in sent), sent
print("  GOTO_SAFE -45°(左) → MOVE L（符号正确）")

clk = Clock()
sent.clear()
r = rk._hri_execute(("goto", 45.0), send=send, yaw_fn=lambda: None,
                    obs_fn=lambda: (False, 80), sleep_fn=clk.sleep, now_fn=clk.now)
assert r == "goto_no_ref" and sent == [], (r, sent)
print("  IMU 丢失 → goto_no_ref，不发命令")

print("\n全部断言通过")
