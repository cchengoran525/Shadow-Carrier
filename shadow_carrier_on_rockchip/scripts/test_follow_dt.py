#!/usr/bin/env python3
"""follow 控制环提速/速度上调 回归测试 (无需硬件)
统筹单 10-14: 环率 2.5→10Hz + BASE/MIN 上调 + 拍数常量改按秒
用法: python3 test_follow_dt.py  (断言失败即退出非0)
"""
import importlib.util, sys, os

SCRIPTS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPTS)
spec = importlib.util.spec_from_file_location("follow_controller", os.path.join(SCRIPTS, "follow_controller.py"))
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)

cmds, cur = [], {"box": None}
fc = m.FollowController(lambda c: cmds.append(c), person_provider=lambda: cur["box"])
fc.running = True

def tick_fc(ctrl, dt=0.1):
    """模拟真实10Hz: 强制 dt 让 EMA 按正确系数收敛"""
    import time as _t
    ctrl._last_t = _t.monotonic() - dt
    return ctrl.tick()

def settle(cx=None, h=300, dist=None, ticks=20):
    """连续跑 ticks 拍(每拍dt=0.1), 返回最后一拍命令"""
    fc.dist_fn = (lambda: dist) if dist is not None else None
    out = None
    for _ in range(ticks):
        cmds.clear()
        cur["box"] = None if cx is None else {"cx": cx, "cy": 240, "h": h}
        out = tick_fc(fc)
    return (cmds[-1] if cmds else None), out[2].get("state")

print("== 常量 ==")
for k in ["ALPHA", "ALPHA_REF_DT", "BASE_SPD", "MIN_SPD", "CYCLE_S",
          "LOST_TIMEOUT_S", "SEARCH_BACK_S", "LOW_STREAK_S", "NEAR_STOP_M"]:
    print(f"  {k} = {getattr(m, k)}")

print("\n== 稳态方向映射 (20拍收敛; 死区±50) ==")
for cx, note in [(320, "正中"), (290, "-30 死区内"), (240, "-80 小弧"),
                 (200, "-120 中弧"), (120, "-200 满弧"),
                 (440, "+120 右弧"), (520, "+200 右满弧")]:
    cmd, st = settle(cx=cx)
    print(f"  cx={cx:<4} {note:<12} -> {cmd:<16} {st}")

print("\n== 近距急停 (dist_fn < NEAR_STOP_M 才停; 不再用框高) ==")
for d, note in [(0.5, "0.5m 太近→STOP"), (0.7, "0.7m 边界"), (1.0, "1.0m 正常→走")]:
    cmd, st = settle(cx=320, dist=d)
    print(f"    dist={d}  {note:<14} -> {cmd:<16} {st}")

print("\n== 距离匹配注入 (dist_fn; None=恒速) ==")
for d, note in [(3.0, "3.0m 远→快"), (1.2, "1.2m 目标"), (0.9, "0.9m 近→慢"), (None, "无源→恒速")]:
    cmd, st = settle(cx=320, dist=d)
    print(f"  dist={str(d):<5} {note:<12} -> {cmd}")

print("\n== 丢人流程: 两种环率等价性 (按秒计时) ==")
for dt_s, label in [(0.1, "实机 10Hz"), (0.3, "仿真 3.3Hz")]:
    c = m.FollowController(lambda cc: None, person_provider=lambda: None)
    c.running = True; c._last_t = None
    cur["box"] = None
    marks = {}
    t = 0.0
    for i in range(1, int(6/dt_s)+1):
        import time as _t
        c._last_t = _t.monotonic() - dt_s
        _tag, _val, info = c.tick()
        t += dt_s
        st = info.get("state")
        marks.setdefault(st, round(t, 2))
    order = ["lost", "search_back", "lost_stop"]
    print(f"  {label}: " + " | ".join(f"{k}@{marks.get(k)}s" for k in order if k in marks))

print("\n== 响应时间: 目标跳变(cx 320→140)后进入弧线 ==")
c2cmds = []
fc2 = m.FollowController(lambda c: c2cmds.append(c), person_provider=lambda: cur["box"])
fc2.running = True
cur["box"] = {"cx": 320, "cy": 240, "h": 300}
for _ in range(30): tick_fc(fc2)
cur["box"] = {"cx": 140, "cy": 240, "h": 300}
n_new = None
for i in range(1, 31):
    c2cmds.clear(); tick_fc(fc2)
    if c2cmds and c2cmds[0] != "DIFF L130 R130":
        n_new = i; break

def ticks_to_arc(alpha):
    x = 320.0
    for n in range(1, 100):
        x = alpha * 140 + (1 - alpha) * x
        if abs(x - 320) > 50:
            return n
    return None

n_old = ticks_to_arc(0.3)
print(f"  新版: {n_new}拍 = {n_new*0.1:.1f}s (10Hz, ALPHA=0.18)")
print(f"  旧版: {n_old}拍 = {n_old*0.4:.1f}s (2.5Hz, ALPHA=0.3)")
print(f"  → 反应提速 {n_old*0.4/(n_new*0.1):.1f}x")

print("\n== 断言 (回归护栏) ==")
vals = []
for cx in range(100, 541, 20):
    cmd, _ = settle(cx=cx, ticks=10)
    if cmd and cmd.startswith("DIFF"):
        vals += [int(t[1:]) for t in cmd.split()[1:]]
assert min(vals) > 50, f"轮速落入固件PWM死区: min={min(vals)}"
assert max(vals) <= 160, f"轮速过高: max={max(vals)}"
assert m.LOST_TIMEOUT_S == 2.4 and m.SEARCH_BACK_S == 1.2 and m.LOW_STREAK_S == 3.2
c = m.FollowController(lambda cc: None, person_provider=lambda: None); c.running = True
cur["box"] = None; marks = {}; t = 0.0
for i in range(1, 61):
    c._last_t = __import__("time").monotonic() - 0.3
    _tg, _v, info = c.tick(); t += 0.3
    marks.setdefault(info.get("state"), round(t, 2))
assert marks.get("lost_stop") == 2.4, f"3.3Hz下丢失超时应为2.4s, 实际{marks.get('lost_stop')}"
print(f"  [PASS] 轮速区间 {min(vals)}~{max(vals)} (>50 且 <=160)")
print(f"  [PASS] 丢失超时环率无关 = {marks.get('lost_stop')}s (10Hz与3.3Hz一致)")
_cn, _ = settle(cx=320, dist=0.5, ticks=6)
assert _cn == "STOP", f"dist<NEAR_STOP_M 应 STOP, 实际{_cn}"
_cf, _ = settle(cx=320, dist=1.0, ticks=6)
assert _cf and _cf.startswith("DIFF"), f"dist>=NEAR_STOP_M 应走, 实际{_cf}"
print(f"  [PASS] 近距急停(距离): dist0.5→{_cn}, dist1.0→{_cf}")
print("  全部断言通过")
