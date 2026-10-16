#!/usr/bin/env python3
"""回归: RECEIVE 的"近距站定持物"放行 与 不误收边界。

背景: 主人在车跟前站定递物(v~0)本是常见姿态, 旧判据 `held and slow` 要求 v>=0.06
接近, 导致 hide_occupied(回退停靠点离交接点仅~1m) 永不触发。改为
`held and (slow or (已近<=near_dist_m 且 站定))`。本测试钉住三条边界:

  1) 近距站定 + 持物(bottle)  -> 必须 RECEIVE      (新放行)
  2) 近距站定 + 空手          -> 不得 RECEIVE      (别把"站着"当交接)
  3) owner.source=file 无锁   -> 不得认任何人/RECEIVE (机主原则 L2 不被绕过)

用 sim 真实回放帧的像素框驱动, 距离由 Kw/宽度 反推。
"""
import copy
import json
import os
import time

from hri_state import HRIStateMachine, load_params

HERE = os.path.dirname(os.path.abspath(__file__))

# 取自 reg2_hide_occupied.jsonl 真实回放帧(t=67.7, 站定, dist≈0.89)
PERSON = {"label": "person", "conf": 0.85, "bbox": [115, -309, 525, 789]}
BOTTLE = {"label": "bottle", "conf": 0.9, "bbox": [425, 179, 476, 301]}


def _fast_params(owner_source="conf"):
    p = copy.deepcopy(load_params(os.path.join(HERE, "hri_params.json")))
    p["timing"].update(static_need_s=0.3, dwell_s=0.1, hide_after_s=999.0,
                       receive_cooldown_s=0.1, owner_lost_abort_s=2.0)
    p.setdefault("owner", {})["source"] = owner_source
    if owner_source == "file":
        p["owner"]["file"] = "/dev/shm/__no_such_owner__.json"
    return p


def _drive(dets, seconds=2.0, dt=0.1):
    sm = HRIStateMachine(params=_fast_params(
        "file" if dets is None else "conf"))
    reached = None
    deadline = time.time() + seconds
    while time.time() < deadline:
        out = sm.feed(dets or [PERSON], robot_moving=False)
        if out.get("state") == "RECEIVE":
            reached = out
            break
        time.sleep(dt)
    return reached


def main():
    ok = True

    r1 = _drive([PERSON, BOTTLE])
    print("1) 近距站定+持物   ->", "RECEIVE" if r1 else "no", r1)
    ok &= r1 is not None

    r2 = _drive([PERSON])
    print("2) 近距站定+空手   ->", "RECEIVE" if r2 else "no")
    ok &= r2 is None

    r3 = _drive(None)  # source=file 且无锁
    print("3) file 无锁       ->", "RECEIVE" if r3 else "no")
    ok &= r3 is None

    print("=== %s ===" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
