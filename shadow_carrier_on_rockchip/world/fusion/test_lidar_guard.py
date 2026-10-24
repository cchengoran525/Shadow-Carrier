#!/usr/bin/env python3
"""test_lidar_guard.py —— 雷达避障核回归夹具（离线, 无需硬件/雷达）

把"避障该有的行为"固化成断言: 迟滞不抖 / 硬间隙立即停 / 遮罩与近场自反射 /
腿级行人提前量 / 飞点丢弃 / stale 降级 / 命令过滤语义 / 方位角符号约定。
用法: python3 test_lidar_guard.py
"""
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lidar_guard as lg   # noqa: E402

PASS, FAIL = [], []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    print("%-4s %s %s" % ("PASS" if cond else "FAIL", name, "" if cond else detail))


# ---------------- 合成扫描工具 ----------------

def ring(r, step=2, lo=0, hi=360):
    """整圈均匀回波(空场/远墙)"""
    return [[b % 360, r] for b in range(lo, hi, step)]


def wall(dist, half_deg=40.0, step=1.0):
    """正前 ±half_deg 的一堵墙(垂直入射, 距离 = dist/cos)"""
    return [[b, dist / max(1e-6, math.cos(math.radians(b)))]
            for b in [i * step for i in range(-int(half_deg / step), int(half_deg / step) + 1)]]


def side_wall(dist, center_deg, width_deg=20.0, step=1.0):
    """指定方位的一小段墙"""
    return [[center_deg + d, dist / max(1e-6, math.cos(math.radians(d)))]
            for d in [i * step for i in range(int(-width_deg / 2), int(width_deg / 2) + 1)]]


def blob(center_deg, dist, width_m=0.10, step=1.5):
    """点状目标(腿/柱): 宽 width_m 的一小簇点"""
    half = math.degrees(math.atan2(width_m / 2.0, dist))
    return [[center_deg + d, dist / max(1e-6, math.cos(math.radians(d)))]
            for d in [i * step for i in range(int(-half / step), int(half / step) + 1)]]


def feed(guard, points, n=6, dt=0.1, ts_lag=0.05):
    """连续喂 n 帧(推进时间), 返回最后一帧包络"""
    env = None
    now = time.monotonic()
    for i in range(n):
        now += dt
        env = guard.update({"ts": now - ts_lag,
                            "points": points}, now=now)
    return env


CFG = lg.load_config()

# ============ T1 空场: 全清, 给满速度 ============
g = lg.Guard(CFG)
env = feed(g, ring(6.0))
check("T1 空场不设限", env["blocked"]["fwd"] is False and env["limits"]["vx_max"] > 0.55
      and env["mode"] == "ok", str(env["limits"]))

# ============ T2 正前墙: 分级限速 → 停 ============
for dist, exp_lim, exp_blocked in ((0.50, 0.38, False), (0.30, None, True), (0.20, None, True)):
    gg = lg.Guard(CFG)
    e = feed(gg, wall(dist) + ring(6.0, lo=60, hi=300))
    vx = e["limits"]["vx_max"]
    if exp_lim is None:
        check("T2 正前墙 %.2fm → blocked" % dist, e["blocked"]["fwd"] is True, str(e["limits"]))
    else:
        check("T2 正前墙 %.2fm → 限速≈%.2f" % (dist, exp_lim),
              (not e["blocked"]["fwd"]) and abs(vx - exp_lim) < 0.06, "vx=%s" % vx)

# 硬间隙(已进车体+5cm) 必须【首帧】就否决, 不被迟滞拖延
g = lg.Guard(CFG)
e = g.update({"ts": time.monotonic(), "points": wall(0.20) + ring(6.0, lo=60, hi=300)})
check("T2b 硬间隙首帧立即否决(不等确认窗)", e["blocked"]["fwd"] is True,
      str(e["gap_m"]))
check("T2b2 否决后 vx 上限为 0", e["limits"]["vx_max"] == 0.0, str(e["limits"]))

# ============ T3 迟滞: 单帧噪声不得推动整车 ============
g = lg.Guard(CFG)
e = g.update({"ts": time.monotonic(), "points": wall(0.30) + ring(6.0, lo=60, hi=300)})
check("T3 单帧近墙不立刻判 blocked(确认窗 0.25s)", e["blocked"]["fwd"] is False,
      str(e["blocked"]))
e = feed(g, wall(0.30) + ring(6.0, lo=60, hi=300), n=3, dt=0.1)
check("T3b 持续 0.3s → 判 blocked", e["blocked"]["fwd"] is True, str(e["blocked"]))
e = feed(g, ring(6.0), n=2, dt=0.1)
check("T3c 恢复也需持续(0.2s 内不解除)", e["blocked"]["fwd"] is True, str(e["blocked"]))
e = feed(g, ring(6.0), n=8, dt=0.1)
check("T3d 持续空旷 0.8s → 解除", e["blocked"]["fwd"] is False, str(e["blocked"]))

# ============ T4 侧向: 麦轮横移的方向性 ============
# 左侧 0.45m 一堵墙(方位 -90°): 横向(左)受限, 前向不受限
g = lg.Guard(CFG)
pts = side_wall(0.45, -90.0, width_deg=20.0) + ring(6.0, lo=-60, hi=60) \
    + ring(6.0, lo=120, hi=300)
e = feed(g, pts)
check("T4 左侧近墙 → vy_min 受限", e["limits"]["vy_min"] > -0.55 and e["gap_m"]["left"] < 0.31,
      "vy_min=%s gap_left=%s" % (e["limits"]["vy_min"], e["gap_m"]["left"]))
check("T4b 左墙不误伤前向", e["blocked"]["fwd"] is False and e["gap_m"]["fwd"] > 3.0,
      str(e["gap_m"]))

# ============ T5 原地转: 四周贴死 → 禁止转 ============
g = lg.Guard(CFG)
e = feed(g, ring(0.28, step=6))
check("T5 四周 0.28m → 禁止转", e["blocked"]["rot"] is True
      and e["limits"]["wz_max_dps"] == 0.0, str(e["limits"]))
g = lg.Guard(CFG)
e = feed(g, ring(0.55, step=6))
check("T5b 四周 0.55m → 允许转", e["blocked"]["rot"] is False
      and e["limits"]["wz_max_dps"] >= 45.0, str(e["limits"]))
# 侧向 0.30m 双墙 → 转受限但未禁(分级)
g = lg.Guard(CFG)
e = feed(g, side_wall(0.30, -90.0, width_deg=30.0) + side_wall(0.30, 90.0, width_deg=30.0)
         + ring(6.0, lo=-60, hi=60) + ring(6.0, lo=120, hi=300))
check("T5c 侧向 0.30m → 转向分级压缩(未禁)", 0 < e["limits"]["wz_max_dps"] < 45.0
      and e["blocked"]["rot"] is False, str(e["limits"]))

# ============ T6 自遮挡: 铜柱方位 + 近场自反射 ============
pillar_b = CFG["_pillar_bearings"][0]        # ≈ +36.9°(右前柱)
check("T6 铜柱方位被自动遮罩", lg.is_masked(pillar_b, CFG)
      and not lg.is_masked(0.0, CFG), "pillar_b=%.1f masks=%s" % (pillar_b, lg.mask_spans_deg(CFG)))
check("T6a 铜柱回波在预处理即被剔除",
      lg.to_robot_frame([[pillar_b, 0.6]], CFG) == [], str(lg.to_robot_frame([[pillar_b, 0.6]], CFG)))
# 近场(0.28m)的柱位回波 → 不得影响裁决: 与"只有整圈远墙"的包络逐字段一致
g = lg.Guard(CFG)
e_ref = feed(g, ring(6.0))
g = lg.Guard(CFG)
e = feed(g, [[pillar_b, 0.28], [pillar_b + 1.0, 0.28]] + ring(6.0))
check("T6b 柱位近距回波不影响裁决(与纯远墙包络一致)",
      e["n_points"] == e_ref["n_points"] and e["blocked"] == e_ref["blocked"]
      and e["limits"] == e_ref["limits"],
      "n=%s vs %s" % (e["n_points"], e_ref["n_points"]))
# 对照组: 非遮罩方位的同距回波 → 必须生效(禁转)
g = lg.Guard(CFG)
e = feed(g, blob(0.0, 0.28, width_m=0.02) + ring(6.0, lo=20, hi=340))
check("T6c 同距非遮罩方位 → 生效(禁转)", e["blocked"]["rot"] is True, str(e["blocked"]))
# 近场自反射: 0.15m 全向回波(板边/柱体) 应被 range_min 滤掉
g = lg.Guard(CFG)
e = feed(g, ring(0.15, step=6))
check("T6d 0.15m 全向自反射被近场滤波", e["n_points"] == 0 and e["blocked"]["fwd"] is False,
      "n=%s" % e["n_points"])

# ============ T7 失效形态: 无回波 / 空帧不是"全清" ============
g = lg.Guard(CFG)
e = feed(g, ring(7.9, step=2))
check("T7 满量程回波(开阔) → 全清且健康", e["ok"] is True and e["blocked"]["fwd"] is False,
      str(e["mode"]))
# 玻璃/探头被挡: SDK 照常出帧但几乎无回波 → 不得假装"前方畅通"
g = lg.Guard(CFG)
e = feed(g, [[0.0, 7.9], [120.0, 7.9]])
check("T7b 有效点过少 → mode=suspect(不假装全清)", e["mode"] == "suspect" and e["ok"] is False,
      "mode=%s n=%s" % (e["mode"], e["n_points"]))
cmd, note = lg.apply_to_ascii("MOVE F 200", e, CFG)
check("T7c suspect → 限速(保守兜底)", cmd.startswith("MOVE F") and int(cmd.split()[2]) < 200,
      "%s (%s)" % (cmd, note))
# 全空帧(探头死/被完全遮住)
g = lg.Guard(CFG)
e = feed(g, [])
check("T7d 空帧 → suspect 且不越权停车", e["mode"] == "suspect"
      and e["blocked"]["fwd"] is False, "mode=%s" % e["mode"])

# ============ T8 飞点: 远处孤立点丢弃, 近处保守保留 ============
dense = [(float(b), 6.0) for b, _r in ring(6.0)]
# 中位差分支: 3m 孤立点夹在 6m 环里(同角另有点 → 先验同角去重, 再验飞点剔除)
kept = lg.to_robot_frame([(0.0, 5.0)] + dense, CFG)          # 同角: 留更近的 5.0
check("T8 同角双点取更近(单回波雷达物理)", any(abs(p[1] - 5.0) < 0.1 and abs(p[0]) < 1.0
                                              for p in kept), "kept=%d" % len(kept))
# 3m 的孤立点(两侧 2° 处是 6m 环) → 中位差分支丢弃
iso = [(0.0, 3.0)] + [(float(b), 6.0) for b in range(-178, 180, 2)]
kept = lg.to_robot_frame(iso, CFG)
check("T8a 远处孤立飞点(中位差)被丢弃",
      not any(abs(p[1] - 3.0) < 0.1 and abs(p[0]) < 1.0 for p in kept), "kept=%d" % len(kept))
# 无邻点分支: 孤立点周围 12° 空窗(稠密帧) → 丢
sparse = [(30.0, 3.0)] + [(float(b), 6.0) for b, _r in ring(6.0) if not (24.0 <= b <= 36.0)]
kept = lg.to_robot_frame(sparse, CFG)
check("T8b 远处无邻点孤立点被丢弃",
      not any(abs(lg._ang_diff(p[0], 30.0)) < 1.0 for p in kept), "kept=%d" % len(kept))
g = lg.Guard(CFG)
e = feed(g, [[0.0, 0.80]] + ring(6.0, lo=6, hi=354))
check("T8c 近处孤立点保守保留", e["gap_m"]["fwd"] is not None and e["gap_m"]["fwd"] < 0.7,
      "gap_fwd=%s" % e["gap_m"]["fwd"])
# 稀疏帧(点少)不做孤立点剔除 → 真实少回波场景不误删
few = [(0.0, 3.0)] + [(float(b), 6.0) for b in range(45, 360, 45)]
check("T8d 稀疏帧保留孤立点(不误删)", len(lg.to_robot_frame(few, CFG)) == len(few),
      "kept=%d" % len(lg.to_robot_frame(few, CFG)))

# ============ T9 腿级行人与提前量 ============
g = lg.Guard(CFG)
e = feed(g, blob(2.0, 2.0, width_m=0.12) + ring(6.0, lo=20, hi=340))
check("T9 细宽簇判为腿级 person", len(e["person_legs"]) == 1, str(e["person_legs"]))
check("T9b 静止腿不设限(2m 外)", e["limits"]["vx_max"] > 0.55, str(e["limits"]))
# 走近: 距离逐帧缩小 → closing>0 → 动态 + 提前量收紧速度上限
g = lg.Guard(CFG)
now = time.monotonic()
env = None
for i, d in enumerate([2.0, 1.7, 1.4, 1.1, 0.9, 0.8]):
    now += 0.1
    env = g.update({"ts": now - 0.05,
                    "points": blob(0.0, d, width_m=0.12) + ring(6.0, lo=20, hi=340)}, now=now)
check("T9c 逼近的腿标为动态+closing>0", env["nearest"]["dynamic"] is True
      and env["nearest"]["closing_mps"] > 0.5, str(env["nearest"]))
check("T9d 提前量使速度上限低于同距静止情形",
      env["limits"]["vx_max"] < lg.v_from_gap(0.8 - 0.17, CFG) + 1e-9,
      "vx=%s 静态=%s" % (env["limits"]["vx_max"], round(lg.v_from_gap(0.63, CFG), 3)))

# ============ T10 方位角符号约定(与世界线同源: 0=前, 正=右) ============
g = lg.Guard(CFG)
e = feed(g, [[90.0, 0.5]] + ring(6.0, lo=100, hi=350) + ring(6.0, lo=0, hi=80))
check("T10 正右(+90°)目标只压右侧通道",
      e["gap_m"]["right"] < 0.4 and e["gap_m"]["left"] > 3.0, str(e["gap_m"]))
sec_r, sec_n = lg.sector_minima(lg.to_robot_frame([[0.0, 1.0], [30.0, 1.0], [-30.0, 1.0]], CFG), CFG)
check("T10b 扇区0=正前, 扇区号向右递增(24扇区: +30°→2, -30°→22)",
      sec_n[0] == 1 and sec_n[2] == 1 and sec_n[22] == 1, str(sec_n))

# ============ T11 stale 降级: 不装死, 也不裸奔 ============
g = lg.Guard(CFG)
old = time.monotonic() - 3.0
env_old = g.update({"ts": old, "points": ring(6.0)}, now=time.monotonic())
check("T11 扫描过期 → mode=stale / ok=False", env_old["mode"] == "stale"
      and env_old["ok"] is False, str(env_old["mode"]))
cmd, note = lg.apply_to_ascii("MOVE F 180", env_old, CFG)
check("T11b stale → 限速而非停车", cmd.startswith("MOVE F") and int(cmd.split()[2]) < 180,
      "%s (%s)" % (cmd, note))

# T11c/T11d: ts 双时钟兼容(2026-10-24 实测发现): 生产者若写 epoch(time.time(),
# 旧文档 §5.1 示例值), 不得让 stale 检测静默失效(见 V2_LIDAR_AVOIDANCE.md 附 D)
g = lg.Guard(CFG)
env_ep = g.update({"ts": time.time() - 3.0, "points": ring(6.0)}, now=time.monotonic())
check("T11c epoch 时钟的过期帧 → 同样判 stale", env_ep["mode"] == "stale", str(env_ep["mode"]))
g = lg.Guard(CFG)
env_ep2 = g.update({"ts": time.time() - 0.05, "points": ring(6.0)}, now=time.monotonic())
check("T11d epoch 时钟的新鲜帧 → mode=ok 且 scan_age 正常", env_ep2["mode"] == "ok"
      and env_ep2["scan_age_s"] is not None and env_ep2["scan_age_s"] <= 0.6,
      "%s age=%s" % (env_ep2["mode"], env_ep2["scan_age_s"]))

# ============ T12 命令过滤: 四种命令语义 ============
g = lg.Guard(CFG)
e_blk = feed(g, wall(0.30) + ring(6.0, lo=60, hi=300))
cmd, note = lg.apply_to_ascii("MOVE F 180", e_blk, CFG)
check("T12 前方受阻 → MOVE F 降为 STOP", cmd == "STOP", "%s (%s)" % (cmd, note))
cmd, note = lg.apply_to_ascii("MOVE B 150", e_blk, CFG)
check("T12b 后方通畅 → MOVE B 原样放行", cmd == "MOVE B 150", cmd)
cmd, note = lg.apply_to_ascii("DIFF L130 R130", e_blk, CFG)
check("T12c 前方受阻 → DIFF 前进降为 STOP", cmd == "STOP", "%s (%s)" % (cmd, note))
cmd, note = lg.apply_to_ascii("PAN 90.0", e_blk, CFG)
check("T12d 云台命令不受避障影响", cmd == "PAN 90.0", cmd)

# 受限(未 blocked): 分量分解缩放 —— 共模(前进)按速度包络, 差模(转向)按转向包络
g2 = lg.Guard(CFG)
e_lim = feed(g2, wall(0.50) + ring(6.0, lo=60, hi=300))
cmd, note = lg.apply_to_ascii("DIFF L200 R100", e_lim, CFG)
l, r = (int(x[1:]) for x in cmd.split()[1:])
cap_pwm = e_lim["limits"]["vx_max"] / (CFG["v_at_spd_max"] / CFG["spd_max"])
check("T12e 受限 → 前进分量压到包络(转向分量保留)",
      abs((l + r) / 2.0 - cap_pwm) < 1.5 and abs((l - r) / 2.0 - 50) < 1.5,
      "%s (原 L200 R100 cap=%.0fPWM, %s)" % (cmd, cap_pwm, note))

# 转向受限: 差速分量被压缩（手工构造包络, 隔离语义, 不依赖几何）
env_rot = {"mode": "ok", "limits": {"vx_max": 1.0, "vx_min": -1.0, "vy_max": 1.0,
                                    "vy_min": -1.0, "wz_max_dps": 18.0},
           "blocked": {}, "gap_m": {}}
cmd, note = lg.apply_to_ascii("DIFF L160 R100", env_rot, CFG)
l, r = (int(x[1:]) for x in cmd.split()[1:])
check("T12f 转向受限 → 差速分量压缩(保持均值)",
      abs(l - r) < 0.25 * 60 and abs((l + r) / 2.0 - 130.0) < 1.5, "%s (%s)" % (cmd, note))
env_rot2 = dict(env_rot, blocked={"rot": True}, limits=dict(env_rot["limits"], wz_max_dps=0.0))
cmd, note = lg.apply_to_ascii("DIFF L160 R100", env_rot2, CFG)
l, r = (int(x[1:]) for x in cmd.split()[1:])
check("T12f2 禁止转 → 差速归零(直行)", abs(l - r) <= 2 and l > 100, "%s (%s)" % (cmd, note))

# MEC: 逐轴钳位(V2 麦轮语义, 线上单位 cm/s 与 rad/s)
cmd, note = lg.apply_to_ascii("MEC 60.0 60.0 1.5", e_lim, CFG)
vx_cm, vy_cm, wz_rad = (float(t) for t in cmd.split()[1:])
check("T12g MEC vx 被包络钳位", vx_cm <= e_lim["limits"]["vx_max"] * 100 + 1e-6
      and cmd.startswith("MEC") and vy_cm > 10.0,
      "%s (cap=%.2fm/s)" % (cmd, e_lim["limits"]["vx_max"]))

# 无包络 → 命令原样(惰性: 雷达没上线时零影响)
check("T12h env=None 时命令原样放行(惰性契约)",
      lg.apply_to_ascii("MOVE F 180", None, CFG)[0] == "MOVE F 180")


# MOVE L/R = 原地旋转语义(V1 固件): 受转向净空约束, 不受横向走廊约束
env_rotv = {"mode": "ok", "limits": {"vx_max": 1.0, "vx_min": -1.0, "vy_max": 1.0,
                                     "vy_min": -1.0, "wz_max_dps": 0.0},
            "blocked": {"rot": True}, "gap_m": {"left": 5.0}, "rot_gap_m": 0.02}
cmd, note = lg.apply_to_ascii("MOVE L 70", env_rotv, CFG)
check("T12i 原地转净空不足 → MOVE L 降为 STOP", cmd == "STOP", "%s (%s)" % (cmd, note))
env_lat = {"mode": "ok", "limits": {"vx_max": 1.0, "vx_min": -1.0, "vy_max": 1.0,
                                    "vy_min": -0.2, "wz_max_dps": 90.0},
           "blocked": {"left": True}, "gap_m": {"left": 0.1}, "rot_gap_m": 1.5}
cmd, note = lg.apply_to_ascii("MOVE L 70", env_lat, CFG)
check("T12j 左侧走廊受阻不影响原地转(MOVE L 仍执行)", cmd == "MOVE L 70", "%s (%s)" % (cmd, note))
cfg_lat = lg.load_config({"move_lr_is_rotation": False})
cmd, note = lg.apply_to_ascii("MOVE L 70", env_lat, cfg_lat)
check("T12k 麦轮模式(false) 下 MOVE L = 真横移 → 受走廊约束(左侧受阻即停)",
      cmd == "STOP", "%s (%s)" % (cmd, note))
env_lat2 = dict(env_lat, blocked={}, limits=dict(env_lat["limits"], vy_min=-0.15))
cmd, note = lg.apply_to_ascii("MOVE L 70", env_lat2, cfg_lat)
check("T12l 横移净空不足 → 按横向包络限速",
      cmd.startswith("MOVE L") and 30 <= int(cmd.split()[2]) <= 45, "%s (%s)" % (cmd, note))

env_zero = {"mode": "ok", "limits": {"vy_max": 1.0, "vy_min": -1.0, "wz_max_dps": 0.0},
            "blocked": {}, "gap_m": {}, "rot_gap_m": 0.01}
cmd, note = lg.apply_to_ascii("MOVE L 70", env_zero, CFG)
check("T12m 转向包络=0(确认窗内) → 也立即停, 不许转",
      cmd == "STOP", "%s (%s)" % (cmd, note))

# ============ T13 绕行提示(V2 麦轮横移用) ============
g = lg.Guard(CFG)
pts = wall(0.28) + side_wall(0.5, 90.0, width_deg=30.0) + ring(6.0, lo=-60, hi=-10) \
    + ring(6.0, lo=130, hi=300)
e = feed(g, pts)
check("T13 前方受阻+右侧更挤 → 提示向左", e["escape"] is not None
      and e["escape"]["dir"] == "left", "%s gap=%s" % (e["escape"], e["gap_m"]))
check("T13b 提示带可用速度(符号向左为负 vy)",
      e["escape"]["vy_mps"] < 0 and abs(e["escape"]["vy_mps"]) >= CFG["v_min_mps"],
      str(e["escape"]))

# ============ T14 §14.4 grid.json 健康片段 ============
g = lg.Guard(CFG)
e = feed(g, ring(6.0))
snip = lg.grid_sensors(e)
check("T14 sensors.lidar 片段(§14.4 兼容)",
      snip["lidar"]["ok"] is True and snip["lidar"]["age_s"] is not None, json.dumps(snip))
check("T14b 无雷达时片段给出 ok=False/off",
      lg.grid_sensors(None)["lidar"]["mode"] == "off")

# ============ T15 对称性/回归: 走廊场景 ============
# 走廊 1.1m 宽(左右各 0.55m) + 前方 3m 处横墙 → 前向限速但不 blocked, 可转
g = lg.Guard(CFG)
cor = side_wall(0.55, -90.0, width_deg=180.0, step=3.0) \
    + side_wall(0.55, 90.0, width_deg=180.0, step=3.0) + wall(3.0, half_deg=30.0)
e = feed(g, cor)
check("T15 走廊: 前向通畅且可横移", e["blocked"]["fwd"] is False
      and e["limits"]["vx_max"] > 0.55 and e["limits"]["vy_max"] > 0.15,
      json.dumps({k: e[k] for k in ("limits", "blocked")}))

print("\n== %d passed, %d failed ==" % (len(PASS), len(FAIL)))
if FAIL:
    print("failed: " + ", ".join(FAIL))
sys.exit(1 if FAIL else 0)
