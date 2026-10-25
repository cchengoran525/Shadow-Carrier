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

print("\n== E. uart_send 失败计数与 /healthz 暴露 (2026-10-24 白名单#3) ==")
_f0 = rk.uart_stats["fail"]
assert rk.uart_send("PING") is False, "uart_fd=None(未打开)时发送应失败"
assert rk.uart_stats["fail"] == _f0 + 1, "失败必须计数(不再静默)"
assert rk.uart_stats["last_cmd"] == "PING"
_h = rk._healthz()   # Mac 上各探测项均带 try/except, 安全
assert isinstance(_h.get("uart"), dict) and _h["uart"]["fail"] >= 1, \
    "/healthz 必须暴露 uart.fail"
print(f"  fail×{rk.uart_stats['fail']} last_cmd={rk.uart_stats['last_cmd']!r}"
      f" · /healthz.uart.fail={_h['uart']['fail']}")

print("\n== F. OBS 兜底: blocked/cm<=35 拦前进, 后退/停止放行 (2026-10-24) ==")
now = time.monotonic()
rk.obs_state.update(t=now, blocked=True, cm=20)
assert rk._obs_guard("MOVE F 200") == "STOP"
assert rk._obs_guard("DIFF L200 R100") == "STOP"
assert rk._obs_guard("DIFF L200 R0") == "STOP"
assert rk._obs_guard("DIFF L0 R0") == "DIFF L0 R0"
assert rk._obs_guard("MOVE B 80") == "MOVE B 80"
assert rk._obs_guard("STOP") == "STOP"
rk.obs_state.update(t=now, blocked=False, cm=30)
assert rk._obs_guard("MOVE F 200") == "STOP"
rk.obs_state.update(t=now, blocked=False, cm=80)
assert rk._obs_guard("MOVE F 200") == "MOVE F 200"
rk.obs_state.update(t=now - 5, blocked=True, cm=10)
assert rk._obs_guard("MOVE F 200") == "MOVE F 200"      # 过期 → 放行
print("  blocked→STOP · cm30→STOP · DIFF前进→STOP · 后退/停止→放行 · 过期→放行")

print("\n== G. /turn 门禁: active 原子占位 + IMU 拒绝不残留 (2026-10-25 §7.1-⑤) ==")
import threading as _th
_orig_uart_send, _orig_read_yaw, _orig_worker = rk.uart_send, rk._read_yaw, rk._turn_worker
_sent_g = []
rk.uart_send = lambda c: (_sent_g.append(c), True)[1]
rk._read_yaw = lambda: 0.0
assert rk._get_mode() == "manual"


def _slow_worker(deg, spd, pivot=False):      # 占住 active 0.4s 的假 worker
    time.sleep(0.4)
    with rk.turn_lock:
        rk.turn_state["active"] = False


rk._turn_worker = _slow_worker
assert rk._turn_start(30) is True
time.sleep(0.05)
assert rk._turn_start(30) is False, "worker 活跃期间必须拒绝(修复前: 无锁检查有双开窗口)"
time.sleep(0.5)

rk._turn_worker = _slow_worker
bar = _th.Barrier(10)
wins = []


def _attempt():
    bar.wait()
    wins.append(rk._turn_start(30))


_ts = [_th.Thread(target=_attempt) for _ in range(10)]
[t.start() for t in _ts]
[t.join() for t in _ts]
assert sum(wins) == 1, f"并发 10 请求应恰好放行 1 个, 实际 {sum(wins)}"
time.sleep(0.5)
with rk.turn_lock:
    assert rk.turn_state["active"] is False

rk._read_yaw = lambda: None                    # IMU 缺失: 拒绝且不得残留 active=True
assert rk._turn_start(30) is False
with rk.turn_lock:
    assert rk.turn_state["active"] is False, "IMU 拒绝路径不得残留 active=True"

rk._read_yaw = lambda: 0.0                     # 真 worker: deg=0 立即到位 → ok + STOP 收尾
rk._turn_worker = _orig_worker
_sent_g.clear()
assert rk._turn_start(0) is True
time.sleep(0.3)
with rk.turn_lock:
    assert rk.turn_state["active"] is False and rk.turn_state["result"] == "ok"
assert _sent_g[-1] == "STOP", _sent_g


def _yaw_once():                               # _turn_start 的门读到 0 后, worker 再读变 None
    rk._read_yaw = lambda: None
    return 0.0


rk._read_yaw = _yaw_once                       # y0=None 竞态窗: 兜底必须判 imu_lost 并清位
assert rk._turn_start(30) is True
time.sleep(0.3)
with rk.turn_lock:
    assert rk.turn_state["active"] is False, "y0=None 兜底必须清 active(防预占位卡死)"
    assert rk.turn_state["result"] == "imu_lost"

rk.uart_send, rk._read_yaw, rk._turn_worker = _orig_uart_send, _orig_read_yaw, _orig_worker
print("  活跃期拒 · 并发10抢1 · IMU拒绝不残留 · deg=0即ok+STOP · y0=None兜底清位")

print("\n== H. gimbal_send NaN/inf 防线 (2026-10-25 §7.1-④ host 侧) ==")
_sent_h = []
rk.uart_send = lambda c: (_sent_h.append(c), True)[1]
_pose0 = dict(rk._last_pose)
assert rk.gimbal_send({"pan": float("nan")}) is False
assert _sent_h == [] and rk._last_pose["pan"] == _pose0["pan"], \
    "NaN pan 不得产生任何命令(修复前: min/max 的 nan 比较序会钳到 PAN_MAX=165 顶死)"
assert rk.gimbal_send({"pan": float("inf")}) is False
assert rk.gimbal_send({"pan": float("nan"), "tilt": 100.0}) is True
assert _sent_h == ["TLT 100.0"], _sent_h
assert rk.gimbal_send({"pan": 45.0, "tilt": float("nan")}) is True
assert _sent_h[-1] == "PAN 45.0", _sent_h
assert rk.gimbal_send({"pan": 200.0}) is True          # 越界仍走正常钳位
assert _sent_h[-1] == f"PAN {rk.PAN_MAX:.1f}", _sent_h
assert json.loads('{"pan": NaN}')["pan"] != json.loads('{"pan": NaN}')["pan"], \
    "前提自检: json.loads 默认接受 NaN(防线必要性的依据)"
rk._last_pose.update(_pose0)
rk.uart_send = _orig_uart_send
print("  NaN/inf 拒收该轴 · 有效轴照发 · 越界钳位不变 · _last_pose 不被污染")

print("\n== I. /motion/config 假调参键摘除 (2026-10-25 统筹批#5) ==")
assert "BACK_SPD" not in rk._FC_KEYS
assert "BACK_SPD" not in rk._motion_get(), \
    "BACK_SPD 在 follow_controller 无任何读取方(backing 恒 False), 不得再作为可调键出现"
import follow_controller as _fcm
_min_spd0 = _fcm.MIN_SPD
_tune0 = rk.MOTION_TUNE
rk.MOTION_TUNE = os.path.join(tmp, "motion_tune.json")   # 持久化打到临时目录
r = rk._motion_set({"BACK_SPD": 200, "MIN_SPD": 160})
assert "BACK_SPD" not in r["applied"] and "BACK_SPD" not in r["config"], r["applied"]
assert r["applied"].get("MIN_SPD") == 160, "真键不受影响"
with open(rk.MOTION_TUNE) as f:
    _saved = json.load(f)
assert "BACK_SPD" not in _saved, "持久化文件不得再含假键"
_fcm.MIN_SPD = _min_spd0
rk.MOTION_TUNE = _tune0
print("  get/set/持久化 三处均无 BACK_SPD · 真键(MIN_SPD)照常生效")

print("\n== J. uart_reader 防自旋退避 (2026-10-25 统筹§11.1) ==")
# 死 fd/EOF 时 os.read 立即返回空(b"")而非阻塞 0.1s —— 删掉旧 else-sleep 后若不加退避,
# while 循环 100% 自旋(恰是 C3 掉线/重枚举场景)。0.25s 内迭代数应为 ~25 量级而非数万。
_read_calls = {"n": 0}
_orig_osread = os.read


def _eof_read(fd, n):
    _read_calls["n"] += 1
    return b""


os.read = _eof_read
rk.uart_fd = 999                     # 假 fd; reader 只在 uart_fd 非 None 时循环
_t = _th.Thread(target=rk.uart_reader, daemon=True)
_t.start()
time.sleep(0.25)
rk.uart_fd = None
_t.join(timeout=1.0)
os.read = _orig_osread
assert not _t.is_alive()
assert _read_calls["n"] < 200, \
    f"死 fd 空读 0.25s 内 {_read_calls['n']} 次迭代 = 忙等(应有 10ms 退避)"
print(f"  J1 死fd空读 0.25s → {_read_calls['n']} 次迭代(退避生效; 无退避时将 >10000)")

# J2: 预占位后 Thread.start() 失败必须回滚(否则 active 永久 True → /turn 永久 busy)
rk._read_yaw = lambda: 0.0


class _BoomThread:
    def __init__(self, *a, **k):
        pass

    def start(self):
        raise RuntimeError("boom")


_orig_thread_cls = rk.threading.Thread
rk.threading.Thread = _BoomThread
try:
    assert rk._turn_start(30) is False, "start 失败应返回 False"
finally:
    rk.threading.Thread = _orig_thread_cls
    rk._read_yaw = _orig_read_yaw
with rk.turn_lock:
    assert rk.turn_state["active"] is False, "start 失败必须回滚预占位"
    assert rk.turn_state["result"] == "thread_fail"
print("  J2 Thread.start 抛异常 → False + active 回滚 + result=thread_fail")

print("\n全部断言通过")
