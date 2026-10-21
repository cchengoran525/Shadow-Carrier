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
m.TURN_ENABLED = False      # 本测试只验弧线/直行包络; 转向通道由 test_turn_pivot 覆盖

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
# 2026-10-19: 原断言是 max<=160; 速度上调(220/120)时被放宽成 <=255 —— 那等于护栏失效。
# 改写成有意义的包络: 外轮不超过 BASE_SPD、内轮不低于 MIN_SPD、常量自身留量程余量。
assert min(vals) >= m.MIN_SPD, f"内轮低于 MIN_SPD: min={min(vals)}"
assert max(vals) <= m.BASE_SPD, f"外轮超过 BASE_SPD: max={max(vals)}"
assert m.BASE_SPD < 255, "BASE_SPD 应留量程余量(别顶满 255)"
assert m.MIN_SPD > 50, "MIN_SPD 应在固件死区(PWM<50)之上"
assert m.BASE_SPD - m.MIN_SPD <= 120, f"弧线差速过大(转向过猛): {m.BASE_SPD - m.MIN_SPD}"
assert m.LOST_TIMEOUT_S == 2.4 and m.SEARCH_BACK_S == 1.2 and m.LOW_STREAK_S == 3.2
c = m.FollowController(lambda cc: None, person_provider=lambda: None); c.running = True
cur["box"] = None; marks = {}; t = 0.0
for i in range(1, 61):
    c._last_t = __import__("time").monotonic() - 0.3
    _tg, _v, info = c.tick(); t += 0.3
    marks.setdefault(info.get("state"), round(t, 2))
assert marks.get("lost_stop") == 2.4, f"3.3Hz下丢失超时应为2.4s, 实际{marks.get('lost_stop')}"
print(f"  [PASS] 轮速包络 {min(vals)}~{max(vals)} (MIN≤·≤BASE={m.BASE_SPD}, 差速≤120)")
print(f"  [PASS] 丢失超时环率无关 = {marks.get('lost_stop')}s (10Hz与3.3Hz一致)")
_cn, _ = settle(cx=320, dist=0.5, ticks=6)
assert _cn == "STOP", f"dist<NEAR_STOP_M 应 STOP, 实际{_cn}"
_cf, _ = settle(cx=320, dist=1.0, ticks=6)
assert _cf and _cf.startswith("DIFF"), f"dist>=NEAR_STOP_M 应走, 实际{_cf}"
print(f"  [PASS] 近距急停(距离): dist0.5→{_cn}, dist1.0→{_cf}")

print("\n== 速度门 + 近距门的分工 (2026-10-19) ==")
_off = m.NEAR_GATE_OFFSET
_front, _ = settle(cx=320 + 50, ticks=8)                    # 正前(死区外一点)
assert _front == f"DIFF L{m.BASE_SPD} R{m.BASE_SPD}", f"正前应全速直行: {_front}"
_side, _ = settle(cx=320 + _off + 130, ticks=10)            # ≈24°: 速度门生效
_side_max = max(int(t[1:]) for t in _side.split()[1:]) if _side.startswith("DIFF") else None
assert _side_max is not None and _side_max < m.BASE_SPD, f"侧前未降速: {_side}"
assert _side_max >= m.MIN_SPD, f"侧前降过头: {_side}"
print(f"  [PASS] 速度门: 正前 {_front} → 侧前(≈24°) {_side} (外轮 < BASE)")
# 距离源存在时: 任意方位角都能急停(dist_fn 路径不受 offset 门限制)
_stop_d, _ = settle(cx=320 + _off + 130, dist=0.5, ticks=6)
assert _stop_d == "STOP", f"有距离源时侧前 <0.7m 应 STOP, 实际 {_stop_d}"
print(f"  [PASS] dist_fn 路径覆盖任意方位角: 侧前 dist=0.5 → {_stop_d}")
# 无距离源时: 框高门只在正前锥生效(passby 不在正前时不能误停)
_side_h, _ = settle(cx=320 + 230, h=450, ticks=8)
assert _side_h and _side_h.startswith("DIFF"), f"侧前大框不应触发框高急停: {_side_h}"
_front_h, _ = settle(cx=320 + 50, h=450, ticks=8)
assert _front_h == "STOP", f"正前大框应触发框高急停: {_front_h}"
print(f"  [PASS] 框高门只认正前: 侧前 → {_side_h}, 正前 → {_front_h}")

print("\n== 框高突变门: 单帧异常忽略 / 持续变化重播种 (2026-10-19 修 bug) ==")
fc.dist_fn = None
cur["box"] = {"cx": 320, "cy": 240, "h": 300}
for _ in range(6):
    tick_fc(fc)
_sh0 = fc.sh
cur["box"] = {"cx": 150, "cy": 240, "h": 470}      # 单帧大跳(弯腰/蹲下那种)
tick_fc(fc)
assert abs(fc.sh - _sh0) < 1e-6, f"单帧高度突跳不应改变平滑值: {fc.sh} vs {_sh0}"
for _ in range(2):                                  # 连续第 3 拍 → 视为真实远近变化
    tick_fc(fc)
assert abs(fc.sh - 470) < 1e-6, f"持续高度变化应重播种, sh={fc.sh}"
assert abs(fc.scx - 150) < 1e-6, f"重播种应同时更新 scx, scx={fc.scx}"
print(f"  [PASS] 单帧跳变保持 {_sh0:.0f}; 持续 3 拍重播种 → scx={fc.scx:.0f} sh={fc.sh:.0f}")

print("  全部断言通过")
