#!/usr/bin/env python3
"""test_turn_pivot.py — 底盘转向通道 (v5) 离线回归, 无需硬件

覆盖 (plan v3 §6.1):
  A. 正常跟走 → 原地转进入次数 = 0   ← 护栏: 防 2026-08 "直行-坦克-直行-坦克" 回归
  B. 横移 → E1 预测式触发 (角速度触发, 非角度)
  C. 贴边 → E2 兜底触发; 回锥内 → 退出回弧线
  D. 归因: 自身在转 + 静止的人 → 不得误判为"人在往外走"
  E. 强线索断 + 弱线索在 → 继续原地转 (只维持不发起)
  F. 无强确认的弱线索 → 被拒
  G. 定向扫描: 90° 一档 + 停顿, 累计 180° 后交回原丢失流程
  H. 方向反转看门狗 → 强制回弧线
  I. 超声波否决; 转向预算
  J. TURN_ENABLED=False → 行为与 v4 一致 (零副作用)

用法: python3 test_turn_pivot.py   (断言失败退出非0)
"""
import importlib.util
import math
import os
import sys

SCRIPTS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPTS)
spec = importlib.util.spec_from_file_location(
    "follow_controller", os.path.join(SCRIPTS, "follow_controller.py"))
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

FW = m.FW
FCX = FW / 2
FX = 508.2            # 与 state/calib/params.py 一致
PIVOT_RATE = m.PIVOT_RATE_DPS   # 与生产同口径(190 PWM ≈174°/s); 实机由 pivot_calib 覆盖


class Clock:
    """可控单调钟: 让 PIVOT_DWELL_S / BLIND_MAX_S / WEAK_TTL_S 等时间门可测"""
    def __init__(self, t0=1000.0):
        self.t = t0


class TimeStub:
    def __init__(self, clock):
        self._c = clock

    def monotonic(self):
        return self._c.t

    def sleep(self, s):
        self._c.t += s


class World:
    """假世界: 人在世界系方位 θ_world, 车按指令自转 → 量测 = θ_world - yaw_theta
    约定: θ 与弧线 offset 同号; MOVE R 使 θ 减小(即朝"正 θ 侧"转), yaw_rate 逆时针为正。"""
    def __init__(self, theta_world=0.0, dist_h=300.0):
        self.theta_world = theta_world
        self.h = dist_h
        self.yaw_theta = 0.0
        self.yaw_rate = 0.0
        self.cmds = []

    def send(self, c):
        self.cmds.append(c)

    def box(self):
        th = self.theta_world - self.yaw_theta
        cx = FCX + FX * math.tan(math.radians(th))
        return {"cx": cx, "cy": 240.0, "h": self.h}

    def theta(self):
        return self.theta_world - self.yaw_theta

    def step_dynamics(self, cmd, dt):
        """按上一条指令推进自转。
        符号关系(与真机一致): MOVE R = 顺时针 = IMU 逆时针为正下的负值;
        而 yaw_theta(θ 轴)取反号 —— 因为 θ=max 时 MOVE R 必须使 θ 减小(朝人转)。"""
        self.yaw_rate = 0.0
        if not isinstance(cmd, str):             # STOP/丢失路径的 0/None
            return
        if cmd.startswith("MOVE") and len(cmd.split()) == 3:
            part = cmd.split()
            if part[1] not in ("L", "R"):        # MOVE F/B = 平移, 不改变航向
                return
            d = 1.0 if part[1] == "R" else -1.0
            self.yaw_theta += d * PIVOT_RATE * dt
            self.yaw_rate = -d * PIVOT_RATE
        elif cmd.startswith("DIFF"):
            l = int(cmd.split()[1][1:]); r = int(cmd.split()[2][1:])
            self.yaw_rate = (r - l) * 0.4    # 左轮快=右转=负(量级仅供归因测试)


def make(world, clock, *, turn=True, weak=None, obs=None, yaw=True):
    m.TURN_ENABLED = turn
    m.time = TimeStub(clock)
    fc = m.FollowController(world.send,
                            person_provider=(lambda: (world.box() if world.h > 0 else None)),
                            yaw_rate_fn=(lambda: world.yaw_rate) if yaw else None,
                            yaw_deg_fn=(lambda: world.yaw_theta) if yaw else None,
                            obs_fn=obs, weak_cue_fn=weak)
    fc.running = True
    fc.profile = "test-owner"
    return fc


def tick(fc, world, clock, dt=0.1, hold=None, weak=None):
    """一拍: 跑 tick → 按**实际下发的线上指令**推进假世界 → 返回 (tag, val, info)"""
    clock.t += dt
    n0 = len(world.cmds)
    tag, val, info = fc.tick()
    sent = world.cmds[-1] if len(world.cmds) > n0 else None
    if hold is not None:                     # 强制量测(仿真"人物在动")
        world.theta_world = hold + world.yaw_theta
    world.step_dynamics(sent, dt)
    return tag, val, info


def is_pivot(val):
    """原地转描述符: "R110"/"L110" (与 DIFF 的 "L130R130" 同风格, 非线上指令全文)。
    注意: 丢失/后退路径的 val 是 int 或 None, 一律不是原地转。"""
    return isinstance(val, str) and val[:1] in ("L", "R") and val[1:].isdigit()


# ============ A. 正常跟走 → 零进入 ============
print("== A. 正常跟走 (含缓慢横移) 不应进入原地转 ==")
clk = Clock(); w = World(); fc = make(w, clk)
pivots = 0; phases = set()
for i in range(120):                        # 12s: 人在中心附近缓慢漂移 ±12°
    hold = 12.0 * math.sin(i / 20.0)
    tag, val, info = tick(fc, w, clk, hold=hold)
    phases.add(info.get("phase"))
    pivots += 1 if is_pivot(val) else 0
print(f"  120 拍: 原地转次数={pivots} phases={sorted(phases)} "
      f"末态θ={fc._theta}")
assert pivots == 0, f"正常跟走出现原地转 {pivots} 次 (回归!)"
assert fc.phase == "TRACK", f"正常跟走相异常: {fc.phase}"
print("  [PASS] 零进入 (护栏有效)")

# ============ B. E1 预测式: 横移出画 ============
print("\n== B. 横移(E1 预测式) → 进原地转, 方向朝人消失侧 ==")
clk = Clock(); w = World(); fc = make(w, clk)
entered = None
for i in range(40):
    hold = min(28.0, i * 6.0)               # 0.6s 内从 0° 走到 28°: 世界角速度 100°/s
    tag, val, info = tick(fc, w, clk, hold=hold)
    if fc.phase in ("TURN", "BLIND") and entered is None:
        entered = (i, val, round(w.theta(), 1), val)
        break
print(f"  进入拍={entered[0]} 指令={entered[1]} θ={entered[2]}° 进相={fc.phase}")
assert entered is not None, "横移未触发 E1"
assert entered[1] == f"R{m.PIVOT_PWM}", f"方向/指令错: {entered[1]}"
print("  [PASS] E1 触发, 方向正确 (θ>0 → MOVE R, 与弧线同源约定)")

# ============ C. E2 兜底 + 退出迟滞 ============
print("\n== C. 贴边(E2) → 进原地转; 回锥内 → 退出回弧线 ==")
clk = Clock(); w = World(theta_world=35.0); fc = make(w, clk)
val = info = None
for i in range(8):
    tag, val, info = tick(fc, w, clk)
    if fc.phase == "TURN":
        break
assert fc.phase == "TURN", f"E2 未触发 (θ={fc._theta}, phase={fc.phase})"
print(f"  E2 进入: 指令={val} θ={fc._theta:.1f}°")
# 回到锥内 (θ=10°) 并保持 > PIVOT_DWELL_S
exited = None
for i in range(10):
    tag, val, info = tick(fc, w, clk, hold=10.0)
    if fc.phase == "TRACK":
        exited = (tag, val); break
assert exited is not None, f"未退出原地转 (θ={fc._theta:.1f})"
assert exited[0] == "DIFF", f"退出后应回弧线, 实际 {exited}"
print(f"  [PASS] 退出后回弧线: {exited[0]} {exited[1]}")

# ============ D. 归因 (三层) ============
print("\n== D1. 自身大幅自转 + 静止的人 → 不得误入转向相 ==")
clk = Clock(); w = World(); fc = make(w, clk)
for i in range(6):
    tick(fc, w, clk, hold=0.0)
w.theta_world = 0.0                          # 人在世界系静止
bad = []
for i in range(30):                          # 车 ±60°/s 来回自转 3s (弧线纠偏那种抖动)
    d = 1.0 if (i // 3) % 2 == 0 else -1.0
    w.yaw_theta += d * 60.0 * 0.1
    w.yaw_rate = -d * 60.0                   # IMU 逆时针为正 → 与 yaw_theta 反号
    clk.t += 0.1
    tag, val, info = fc.tick()
    if fc.phase != "TRACK":
        bad.append((i, fc.phase, info.get("rate_world")))
print(f"  误入相={bad[:3]} (共{len(bad)}拍) phase={fc.phase}")
assert not bad, f"静止的人被自身自转误判为出画: {bad[:3]}"
print("  [PASS] 自身 ±60°/s 自转未触发转向 (E1 前置门 + E2 未及)")

print("\n== D2. 车体安静 + 人缓慢移动 → rate_world ≈ 人的真实角速度 ==")
clk = Clock(); w = World(); fc = make(w, clk)
for i in range(6):
    tick(fc, w, clk, hold=0.0)
rates = []
for i in range(120):
    hold = 15.0 * math.sin(i / 8.0)          # 峰值 18.75°/s, |θ|≤15° (不触发 E1/E2)
    tag, val, info = tick(fc, w, clk, hold=hold)
    if i >= 20:
        rates.append(abs(info.get("rate_world") or 0.0))
peak = max(rates)
print(f"  真值峰值 18.8°/s → 估计峰值 {peak:.1f}°/s phase={fc.phase}")
assert 10.0 <= peak <= 34.0, f"θ̇ 通道量级异常: {peak}"
assert fc.phase == "TRACK", f"慢速摆动不应进入转向相: {fc.phase}"
print("  [PASS] θ̇ 通道可用 (量级正确, 慢速摆动不误触发)")

print("\n== D3. 稳态自转的归因: 自身 15°/s + 静止的人 → rate_world ≈ 0 ==")
clk = Clock(); w = World(); fc = make(w, clk)
for i in range(6):
    tick(fc, w, clk, hold=0.0)
w.theta_world = 0.0
rates = []
for i in range(16):                          # 车稳定以 +15°/s 转 (低于 E1 前置门)
    w.yaw_theta += -15.0 * 0.1               # θ 随之增大 15°/s
    w.yaw_rate = 15.0                        # IMU 逆时针为正
    clk.t += 0.1
    tag, val, info = fc.tick()
    if i >= 10:
        rates.append(info.get("rate_world"))
avg = sum(rates) / len(rates)
print(f"  自身 15°/s 自转 + 静止的人 → rate_world 均值 {avg:.1f}°/s (θ={fc._theta:.1f}°)")
assert abs(avg) < 10.0, (f"归因符号/量级错误: rate_world={avg} "
                         f"(若把 IMU_TO_THETA_SIGN 取反即会失败)")
print("  [PASS] 归因锁定 IMU_TO_THETA_SIGN 符号")

# ============ E. 强线索断 + 弱线索在 → 继续转 ============
print("\n== E. 强检测丢失但弱线索在 → 继续原地转(只维持不发起) ==")
clk = Clock(); w = World(theta_world=34.0)
weak = {"dir": "R", "score": 0.7, "src": "leg_band"}
fc = make(w, clk, weak=lambda: weak)
for i in range(6):
    tick(fc, w, clk)
    if fc.phase == "TURN":
        break
assert fc.phase == "TURN", f"E2 未进入: {fc.phase}"
w.h = 0.0                                    # 立刻隐藏强检测: 模拟"只有腿/YOLO 出不了人"
seq = []
for i in range(12):
    tag, val, info = tick(fc, w, clk)
    seq.append((val, info.get("state")))
print(f"  丢人后指令序列: {seq[:7]}")
pivots = [s for s in seq if is_pivot(s[0])]
assert seq[0][1] == "blind", f"丢人后应进 BLIND, 实际 {seq[0]}"
assert len(pivots) >= 4, f"弱线索在却停了: {seq}"
print(f"  [PASS] 弱线索维持原地转 {len(pivots)} 拍 (未因无人框停下; 结束由转向预算收尾)")

# ============ F. 无强确认的弱线索 → 被拒 ============
print("\n== F. 没有近期强确认 → 弱线索不得发起转向 ==")
clk = Clock(); w = World(theta_world=0.0); w.h = 0.0
fc = make(w, clk, weak=lambda: {"dir": "R", "score": 0.9, "src": "leg_band"})
seq = [tick(fc, w, clk)[1] for _ in range(10)]
print(f"  指令序列: {seq}")
assert not any(is_pivot(v) for v in seq), f"弱线索在无强确认时发起了转动: {seq}"
assert all(not isinstance(v, str) or v.startswith("DIFF") for v in seq), \
    f"关闭/无强确认时指令异常: {seq}"
assert fc.phase == "TRACK", f"相异常: {fc.phase}"
print("  [PASS] 弱线索被拒 (机主原则第0层未被放宽)")

# ============ G. 定向扫描 90° 分档 → 180° 用尽 ============
print("\n== G. 定向扫描: 90° 一档 + 停顿, 累计 180° 后交回丢失流程 ==")
clk = Clock(); w = World(theta_world=30.0)
fc = make(w, clk)                            # 无弱线索
for i in range(8):
    tick(fc, w, clk)
    if fc.phase == "TURN":                   # 一进原地转就隐藏强检测(否则快速转会对齐退出)
        break
w.h = 0.0
holds = 0; moves = 0; reached_search = False
for i in range(200):
    tag, val, info = tick(fc, w, clk)
    if info.get("phase") == "SEARCH":
        reached_search = True
    if info.get("state") == "search_hold":
        holds += 1
    if is_pivot(val):
        moves += 1
    if reached_search and info.get("state") in ("lost", "lost_stop", "wait_owner", "search_back"):
        break
print(f"  SEARCH 到达={reached_search} 停顿拍={holds} 转动拍={moves} "
      f"累计扫描={fc._search_spent:.0f}° 末态={info.get('state')}")
assert reached_search, "未进入定向扫描"
assert holds >= 2, f"扫描分档停顿异常: {holds}"
assert fc._search_spent <= m.SEARCH_MAX_DEG + 20.0, f"扫描超预算: {fc._search_spent}"
assert info.get("state") == "lost_stop" or info.get("state") == "wait_owner" \
    or info.get("state") == "lost", f"扫描用尽后未交回丢失流程: {info.get('state')}"
print("  [PASS] 扫描有界(≤180°), 用尽后交回原等待/丢失流程")

# ============ H. 摇摆工况: 行为层不得抖动 + 看门狗机制 ============
print("\n== H1. 人在车后左右冒头(0.4s 一侧) → 轮端不得出现方向抖动 ==")
clk = Clock(); w = World(); fc = make(w, clk)
for i in range(3):
    tick(fc, w, clk, hold=0.0)
dirs = []                                   # (拍号, 方向): 用**拍号**算间隔, 不是列表下标
for i in range(60):
    side = 40.0 if (i // 4) % 2 == 0 else -40.0   # 2026-08 摇摆工况
    tag, val, info = tick(fc, w, clk, hold=side)
    if is_pivot(val):
        dirs.append((i, val[0]))
flips = [(a, b) for a, b in zip(dirs, dirs[1:]) if a[1] != b[1]]
runs = []                                          # 每段同向的起点(拍号)
for idx, d in dirs:
    if not runs or runs[-1][1] != d:
        runs.append((idx, d))
gaps = [(b - a) * 0.1 for (a, _), (b, _) in zip(runs, runs[1:])]   # 段持续时长(秒)
print(f"  轮端方向序列: {''.join(d for _, d in dirs)}  反转={len(flips)} "
      f"每段时长(s)={gaps}")
# 不变式(比"次数"更本质): 换向间隔不得小于最小驻留 —— 2026-08 的摇摆是 0.2s 一翻,
# 有驻留挡着就不可能复现; 持续真摇摆则由反转看门狗兜底(H2 单独验机制)。
assert all(g >= m.PIVOT_DWELL_S - 1e-9 for g in gaps), \
    f"换向快于最小驻留({m.PIVOT_DWELL_S}s): {gaps}"
# 密度上界由驻留本身决定(换向不可能比"每 PIVOT_DWELL_S 一次"更密)
_span = (dirs[-1][0] - dirs[0][0]) * 0.1
assert (len(flips) - 1) * m.PIVOT_DWELL_S <= _span + 1e-9, \
    f"换向密度超过驻留允许: {len(flips)} 次 / {_span:.1f}s"
print("  [PASS] 换向率被最小驻留压住 (远低于 2026-08 的 5Hz 摇摆)")

print("\n== H2. 看门狗机制: 反转超限 → 强制回弧线 ==")
clk = Clock(); w = World(); fc = make(w, clk)
fc._last_strong_t = clk.t
fc._theta = 40.0
fc.phase = "TURN"
fc._turn_started = clk.t
for k in range(6):                                 # 每 0.3s 改一次方向(越过 0.25s 驻留)
    clk.t += 0.3
    fc._set_turn_dir("R" if k % 2 == 0 else "L", clk.t)
print(f"  _fallback_until 已置={fc._fallback_until > clk.t} "
      f"清空反转记录={len(fc._dir_changes)}")
assert fc._fallback_until > clk.t, "看门狗未触发"
tag, val, info = tick(fc, w, clk)
print(f"  看门狗期内指令: {val}")
assert not is_pivot(val), f"看门狗期内仍在原地转: {val}"
print("  [PASS] 看门狗生效 (可回退)")

# ============ I. 超声波否决 + 转向预算 ============
print("\n== I. 超声波否决 → 停; 预算 → 结束 ==")
clk = Clock(); w = World(theta_world=34.0)
fc = make(w, clk, obs=lambda: (True, 15))
seq = []
for i in range(8):
    tag, val, info = tick(fc, w, clk)
    seq.append((val, info.get("state")))
print(f"  受阻时指令: {seq[-3:]}")
assert any(s[1] == "turn_end" for s in seq), f"受阻未结束转向: {seq[-3:]}"

clk = Clock(); w = World(theta_world=20.0); fc = make(w, clk)
for i in range(6):
    tick(fc, w, clk)
spent = None
for i in range(60):
    # 恒定把人按在 40°: 逼出"永远对不上 → 靠预算收尾"这条路径
    tag, val, info = tick(fc, w, clk, hold=40.0)
    if info.get("state") == "turn_end":
        spent = info.get("turn_deg")
        break
print(f"  预算结束: 累计 {spent}° (上限 {m.PIVOT_BUDGET_DEG}°)")
assert spent is not None, "预算未生效"
print("  [PASS] 预算/否决均能收尾")

# ============ J. TURN_ENABLED=False → 零副作用 ============
print("\n== J. TURN_ENABLED=False: 贴边也不发 MOVE (行为与 v4 一致) ==")
clk = Clock(); w = World(theta_world=35.0)
fc = make(w, clk, turn=False)
seq = [tick(fc, w, clk)[1] for _ in range(20)]
print(f"  指令样本: {seq[:4]}")
assert not any(is_pivot(v) for v in seq), f"关闭状态下仍发原地转: {seq[:4]}"
# 关闭时只应出现弧线描述符("L130R80" 这种, 非纯数字后缀)或 None/丢失路径的 int
assert all(v is None or not isinstance(v, str)
           or (v[:1] in ("L", "R") and not v[1:].isdigit()) for v in seq), \
    f"关闭状态指令异常: {seq[:4]}"
print("  [PASS] 总开关有效 (可回退)")

# ============ K. 停驻域 turn_only (HIDE/WAIT 转头接人) ============
print("\n== K. turn_only: 只转头不动轮(不弧线/不扫描) ==")
clk = Clock(); w = World(theta_world=10.0); fc = make(w, clk)
seq = []
for i in range(10):                          # 人在锥内 → 停驻域应保持不动
    clk.t += 0.1
    tag, val, info = fc.tick(turn_only=True)
    seq.append((val, info.get("state")))
print(f"  锥内(θ≈10°): {seq[:3]}")
assert all(v is None for v, _ in seq), f"停驻域不该有运动指令: {seq[:3]}"
assert all(s == "turn_idle" for _, s in seq), f"状态应为 turn_idle: {seq[:3]}"
# 人走到侧方(|θ|≥30°) → 应原地转头
w.theta_world = 40.0
for i in range(8):
    clk.t += 0.1
    tag, val, info = fc.tick(turn_only=True)
    if is_pivot(val):
        break
print(f"  侧方 → 指令={val} phase={fc.phase}")
assert is_pivot(val), "停驻域未转头接人"
assert fc.phase in ("TURN", "BLIND"), f"相位异常: {fc.phase}"
# 人消失且额度用尽 → 停驻域只 STOP, 不进 SEARCH(停着的车不扫 180°)
w.h = 0.0
saw_search = False
for i in range(40):
    clk.t += 0.1
    tag, val, info = fc.tick(turn_only=True)
    if fc.phase == "SEARCH":
        saw_search = True
    if is_pivot(val):
        d = 1.0 if val[0] == "R" else -1.0
        w.yaw_theta += d * PIVOT_RATE * 0.1
        w.yaw_rate = -d * PIVOT_RATE
print(f"  丢人后: phase={fc.phase} 进过 SEARCH={saw_search}")
assert not saw_search, "停驻域不应启动 180° 扫描"
print("  [PASS] 停驻域: 只转头、不乱动、不扫描")

print("\n全部断言通过")
