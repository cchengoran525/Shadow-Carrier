#!/usr/bin/env python3
"""test_heading_hold.py - 航向闭环符号自测 (板上跑, 需 cv2)
预期: yaw_rate=0 → L100R100 ; +5dps(左漂) → 左快右慢(右修) ; -5dps → 反向
"""
import sys

sys.path.insert(0, "/home/kickpi/shadow_carrier_on_rockchip/scripts")
from follow_controller import FollowController  # noqa: E402


def run(yr):
    cmds = []
    fc = FollowController(lambda c: cmds.append(c),
                          person_provider=lambda: {"cx": 320, "cy": 240, "h": 120},
                          yaw_rate_fn=(None if yr is None else (lambda: yr)))
    fc.start()
    fc.tick()
    return cmds[-1] if cmds else "no-cmd"


print("=== 航向闭环符号自测 ===")
for yr in (0.0, +5.0, -5.0, None):
    print(f"yaw_rate={yr if yr is not None else 'None(数据过期)'} → {run(yr)}")
print("预期: 0→L100R100 / +5→L110R90(右修) / -5→L90R110(左修) / None→L100R100(退开环)")
