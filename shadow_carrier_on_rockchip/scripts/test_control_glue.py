#!/usr/bin/env python3
"""test_control_glue.py — rk_control 的粘合层离线回归 (2026-10-19)

覆盖两条"新加但没测试"的代码:
  A. `_manual_forward`  手动 IMU 直线保持 (2026-10-18 新增, 原只有板上脚本)
  B. `_hri_parked`      停驻域判据 (turn_only 的门控输入; 文件缺失/过期必须落回完整跟随)
  C. `_read_yaw` / `_get_yaw_rate` 的注入路径与过期/质量门

全部走注入(不发串口/不读 /dev/shm), 可在 Mac 上跑:
用法: python3 test_control_glue.py
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


def fwd(yaw):
    """跑一次 _manual_forward(注入 yaw) → 返回实际下发的命令"""
    sent.clear()
    rk._manual_forward(send=send, yaw_fn=(lambda: yaw))
    return sent[-1]


def write(name, payload):
    p = os.path.join(tmp, name)
    with open(p, "w", encoding="utf-8") as f:
        json.dump(payload, f)
    return p


print("== A. _manual_forward: IMU 缺失 / 直线保持 / 修正方向 ==")
rk._manual_reset()
c = fwd(None)
assert c == f"MOVE F {rk.MANUAL_FWD_SPD}", c
print(f"  IMU 不可用 → {c} (退固定前进)")

rk._manual_reset()
c1 = fwd(100.0)                                            # 首次: 取目标
assert c1 == f"DIFF L{rk.MANUAL_FWD_SPD} R{rk.MANUAL_FWD_SPD}", c1
# 车头偏到目标左侧(y > target, err<0) → 应右修: 左快右慢
c2 = fwd(105.0)
l2 = int(c2.split()[1][1:]); r2 = int(c2.split()[2][1:])
assert l2 > r2, f"y 超前(偏左)应右修(左快), 实际 {c2}"
assert (l2, r2) == (min(255, rk.MANUAL_FWD_SPD + 15),
                    min(255, rk.MANUAL_FWD_SPD - 15)), c2
# 车头偏右(y < target) → 左修: 左慢右快
c3 = fwd(95.0)
l3 = int(c3.split()[1][1:]); r3 = int(c3.split()[2][1:])
assert l3 < r3, f"y 滞后(偏右)应左修(右快), 实际 {c3}"
print(f"  目标 100°: y=105 → {c2}; y=95 → {c3} (方向正确)")

# 大误差 → 修正被 MH_CORR_MAX 夹住, 且轮速不越界
c4 = fwd(160.0)
l4 = int(c4.split()[1][1:]); r4 = int(c4.split()[2][1:])
assert abs(l4 - r4) <= 2 * rk.MH_CORR_MAX, f"修正未夹住: {c4}"
assert 0 <= l4 <= 255 and 0 <= r4 <= 255, f"轮速越界: {c4}"
print(f"  大误差(y=160) → {c4} (修正夹在 ±{rk.MH_CORR_MAX})")
# 弧度穿越: 目标 179, y=-179 → err=+2 (不是 -358)
rk._manual_reset()
fwd(179.0)
c5 = fwd(-179.0)
l5 = int(c5.split()[1][1:]); r5 = int(c5.split()[2][1:])
assert abs(l5 - r5) <= 2 * rk.MH_CORR_MAX, f"±180 穿越未按短弧处理: {c5}"
print(f"  ±180 穿越(179→-179) → {c5} (按短弧 +2° 处理)")

print("\n== B. _hri_parked: 缺失/过期/未停驻/正常 ==")
p_ok = write("hri_ok.json", {"ts": time.time(), "state": "WAIT", "parked": True})
p_stale = write("hri_stale.json", {"ts": time.time() - 5, "parked": True})
p_run = write("hri_run.json", {"ts": time.time(), "state": "FOLLOW", "parked": False})
assert rk._hri_parked(path=os.path.join(tmp, "nope.json")) is False
assert rk._hri_parked(path=p_ok) is True
assert rk._hri_parked(path=p_stale) is False
assert rk._hri_parked(path=p_run) is False
bad = os.path.join(tmp, "bad.json")
open(bad, "w").write("{not json")
assert rk._hri_parked(path=bad) is False
print("  缺失→False · 新鲜 parked→True · 过期→False · FOLLOW→False · 坏JSON→False")

print("\n== C. _read_yaw / _get_yaw_rate 的注入与门限 ==")
now_mono = time.monotonic()
p_imu = write("imu.json", {"ts": now_mono, "yaw_deg": 12.5, "yaw_rate_dps": -3.0,
                           "quality": 1.0})
assert rk._read_yaw(path=p_imu) == 12.5
assert rk._get_yaw_rate(path=p_imu) == -3.0
p_old = write("imu_old.json", {"ts": now_mono - 5, "yaw_deg": 1.0,
                               "yaw_rate_dps": 1.0, "quality": 1.0})
assert rk._read_yaw(path=p_old) is None and rk._get_yaw_rate(path=p_old) is None
p_bad = write("imu_badq.json", {"ts": now_mono, "yaw_deg": 1.0,
                                "yaw_rate_dps": 1.0, "quality": 0.1})
assert rk._read_yaw(path=p_bad) is None and rk._get_yaw_rate(path=p_bad) is None
print("  新鲜→值 · 过期→None · quality<0.5→None")

print("\n== D. 距离环注入: _get_owner_dist 的边界 ==")
rk._ground_distance_m = None
os.environ.setdefault("X", "1")
try:
    from range_from_box import ground_distance_m as gdm
except Exception:
    gdm = None
if gdm is not None:
    rk._ground_distance_m = gdm
    p_owner = write("owner.json", {"ts": time.time(), "bbox": [100, 150, 300, 400]})
    old = rk.OWNER_JSON
    rk.OWNER_JSON = p_owner
    d = rk._get_owner_dist()
    assert d is not None and 2.0 < d < 3.2, f"底边 400 应 ≈2.6m, 实际 {d}"
    p_clip = write("owner_clip.json", {"ts": time.time(), "bbox": [0, 300, 640, 479]})
    rk.OWNER_JSON = p_clip
    assert rk._get_owner_dist() is None, "底边贴框(近距)应返回 None"
    rk.OWNER_JSON = old
    print(f"  底边 400 → {d} m · 底边贴框 → None (近距退回框高/速度门)")
else:
    print("  (跳过: range_from_box 不可导入)")

print("\n全部断言通过")
