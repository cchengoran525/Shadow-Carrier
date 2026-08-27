#!/usr/bin/env python3
"""bearing_test.py - M1 方位角卷尺验证 (桌面尺度)
约定:
  云台锁死在 (PAN_BASE=79, 任意tilt)。targets.json 记录真值:
    [{"label":"瓶-左20","L_cm":-20,"D_cm":45}, ...]  (L负=物理左侧)
  循环打印: 每个检测目标的 解码方位角 vs 同帧最近真值。
用法: python3 bearing_test.py targets.json
"""
import json
import math
import sys
import time
import urllib.request

sys.path.insert(0, "../calib")
from calib.params import FX, CX, K1, K2  # noqa: E402

from decoder import MIRROR_X, decode_bearings  # noqa: E402


def load_targets(path):
    with open(path) as f:
        t = json.load(f)
    for row in t:
        row["true_deg"] = round(math.degrees(
            math.atan(row["L_cm"] / row["D_cm"])), 2)
        row["done"] = False
    return t


def fetch():
    req = urllib.request.Request(
        "http://127.0.0.1:8080/api/detections",
        headers={"User-Agent": "bearing-test"})
    return json.loads(urllib.request.urlopen(req, timeout=2).read())


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "targets.json"
    targets = load_targets(path)
    print(f"镜像={MIRROR_X} fx={FX:.1f} cx={CX:.1f}")
    print("真值表:")
    for i, t in enumerate(targets):
        print(f"  {i}: L={t['L_cm']:+.0f}cm D={t['D_cm']:.0f}cm "
              f"→ 真值 {t['true_deg']:+.1f}°")

    print("\n实时解码 (Ctrl+C 结束):")
    last_print = 0
    while True:
        try:
            dets = fetch().get("detections", [])
        except Exception as e:
            print(f"API错误: {e}")
            time.sleep(1)
            continue
        now = time.monotonic()
        if dets and now - last_print > 0.8:
            results = decode_bearings(dets)
            for r in results:
                if r.get("c") == "person":
                    continue  # 人不算靶子, 避免干扰
                print(f"  [{r['c']}] u中心="
                      f"{(r['x1']+r['x2'])/2:.0f}px → 方位 {r['bearing_deg']:+.2f}°"
                      f"  (conf {r.get('score', '?')})")
            last_print = now
        time.sleep(0.15)


if __name__ == "__main__":
    main()
