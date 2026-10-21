#!/usr/bin/env python3
"""test_yaw_rate.py — 弧线段 IMU 角速度闭环的符号/阻尼离线测试 (无需硬件)

验证 2026-10-21 的改动: IMU 不再只在正前死区生效, 而是全程用
w_des(期望转向角速度) - yaw_rate 做差速修正。
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import follow_controller as m

m.TURN_ENABLED = False          # 隔离转向通道, 只测弧线/直行段


def run(cx, yaw_rate, h=300):
    cmds = []
    c = m.FollowController(
        cmds.append,
        person_provider=lambda: {"cx": cx, "h": h, "cy": 240},
        yaw_rate_fn=lambda: yaw_rate,
        bearing_of_fn=lambda x: math.degrees(math.atan((x - 320.0) / 508.2)))
    assert c.start(), "start failed"
    c.tick()
    diffs = [x for x in cmds if x.startswith("DIFF")]
    assert diffs, f"no DIFF cmd: {cmds}"
    d = diffs[-1]
    left = int(d.split("L")[1].split("R")[0])
    right = int(d.split("R")[1])
    return left, right, d


def main():
    # 1) 人正前 + 车在逆时针漂(CCW, yaw_rate>0) → 应右修: 左轮快/右轮慢
    l, r, d = run(320, +10.0)
    assert l > r, f"正前 CCW 漂移未右修: {d}"
    print(f"[PASS] 正前 + CCW 漂移 → 右修  {d}")

    # 2) 人偏右(+100px), 车未转 → 应右转(CW): 左轮快/右轮慢
    l2, r2, d2 = run(420, 0.0)
    assert l2 > r2, f"人偏右未右转: {d2}"
    print(f"[PASS] 人偏右 +100px → 右转  {d2}")

    # 3) 人偏左(-100px) → 应左转(CCW): 右轮快/左轮慢
    l3, r3, d3 = run(220, 0.0)
    assert r3 > l3, f"人偏左未左转: {d3}"
    print(f"[PASS] 人偏左 -100px → 左转  {d3}")

    # 4) 人偏右但车已经过转(CW 过快, yaw_rate=-20) → 阻尼, 差速应比 case2 小
    l4, r4, d4 = run(420, -20.0)
    assert (l4 - r4) < (l2 - r2), f"过转未阻尼: case2={l2-r2} case4={l4-r4}"
    print(f"[PASS] 人偏右但 CW 过快 → 阻尼  {d4} (差速 {l4-r4} < {l2-r2})")

    # 5) 正前 + 车无漂 → 左右应基本对称
    l5, r5, d5 = run(320, 0.0)
    assert abs(l5 - r5) <= 2, f"正前无漂移却不直: {d5}"
    print(f"[PASS] 正前无漂移 → 直行  {d5}")

    print("\nALL PASS (5/5)")


if __name__ == "__main__":
    main()
