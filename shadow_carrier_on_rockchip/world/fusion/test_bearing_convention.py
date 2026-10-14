#!/usr/bin/env python3
"""test_bearing_convention.py —— 世界方位角符号链审计（离线，无需硬件）

背景（2026-10-14 统筹派单）：跟人链路事故的符号核查。
约定事实（源码/实测）:
  - 摄像头镜像: 图像 u 增大 = 物理【左】（[云台] 2026-08-27 实测）
  - PAN_SIGN=-1  → pan 指令增大 = 物理【右】（gaze.py:27）
  - [世界] polar_mapper / grid.json 的世界方位角约定 = **右为正**（0=正前）
  - IMU 消费取 -yaw_deg（CCW 为正）

用 3 个物理情形验证 `rk_control` 的 owner_bearing 公式：
  theta = u_to_bearing(u) + (pan - PAN_FORWARD)          ← 现状
  theta = (pan - PAN_FORWARD) - u_to_bearing(u)          ← 建议修正
用法: python3 test_bearing_convention.py
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
STATE = os.path.abspath(os.path.join(HERE, "..", "..", "state"))
sys.path.insert(0, STATE)

from gaze import u_to_bearing, PAN_FORWARD          # noqa: E402  (state/gaze.py)

FX, CX = None, None
try:
    from calib.params import FX as _FX, CX as _CX   # noqa: E402
    FX, CX = _FX, _CX
except Exception:
    FX, CX = 508.2, 292.7                            # 冻结内参兜底

PASS, FAIL = [], []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    print("%-4s %s %s" % ("PASS" if cond else "FAIL", name, "" if cond else detail))


def u_of(phys_deg):
    """物理右为正的角度 → 图像 u（含镜像: u↑=物理左）"""
    return CX - FX * math.tan(math.radians(phys_deg))


def world_truth(phys_deg, pan_err_deg_right):
    return phys_deg + pan_err_deg_right


def cur_formula(u, pan):
    return u_to_bearing(u) + (pan - PAN_FORWARD)


def fix_formula(u, pan):
    return (pan - PAN_FORWARD) - u_to_bearing(u)


cases = [
    # (物理方位, 云台偏角(右正), 说明)
    (+11.0, 0.0, "目标在物理右 11°，云台居中 → 期望 +11°"),
    (-11.0, 0.0, "目标在物理左 11°，云台居中 → 期望 -11°"),
    (+20.0, -8.0, "目标右20°，云台已右偏8° → 期望 +12°"),
]
print("== 世界方位角符号链 ==")
for phys, pan_err, desc in cases:
    u = u_of(phys)                      # 该物理方位对应的像素(镜像)
    pan = PAN_FORWARD + pan_err
    cur, fix = cur_formula(u, pan), fix_formula(u, pan)
    truth = world_truth(phys, pan_err)
    # 目的是"证伪现状 + 证明修正"：现状必须偏离真值, 修正必须命中
    check("现状公式被证伪 %s" % desc, abs(cur - truth) > 2.0,
          "现状=%.1f° 真值=%.1f°" % (cur, truth))
    check("修正公式命中 %s" % desc, abs(fix - truth) < 1.0,
          "修正=%.1f° 真值=%.1f°" % (fix, truth))

# 结论性断言：目标在**图像右侧**（u>cx）= 物理左 → 世界应为负
u_imgright = CX + FX * math.tan(math.radians(11.0))     # u>cx
check("镜像事实: 图像右侧 = 物理左 → u_to_bearing 为正",
      u_to_bearing(u_imgright) > 0, "u_to_bearing=%.1f" % u_to_bearing(u_imgright))
check("BUG 定位: 现状公式把它算成正(物理左正)而非世界负",
      cur_formula(u_imgright, PAN_FORWARD) > 0,
      "现状=%.1f" % cur_formula(u_imgright, PAN_FORWARD))
check("修正公式同情形给出世界负值(右正约定)",
      fix_formula(u_imgright, PAN_FORWARD) < 0,
      "修正=%.1f" % fix_formula(u_imgright, PAN_FORWARD))

print("\n== %d passed, %d failed ==" % (len(PASS), len(FAIL)))
print("结论: rk_control.owner_bearing 现状公式混用了'物理左正'(u_to_bearing) 与"
      "'物理右正'(pan-FORWARD)；应改为 theta=(pan-PAN_FORWARD) - u_to_bearing(u)")
sys.exit(1 if FAIL else 0)
