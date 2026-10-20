#!/usr/bin/env python3
"""passby_record.py — T4 passby 客观记录器 (10Hz 采样 → JSONL)

两种用法:
  1) 命令行(SSH): python3 passby_record.py [--secs N] [--out file]
  2) 网页测试台: rk_control 导入 record() 在线程里跑 (见 /test 页面的 T4 面板)

记录: owner.json 新鲜度/框、/api/detections 的 person 框、imu.json 航向、hri_state.json。
只读, 不控车、不改任何服务。
"""
import argparse
import json
import os
import signal
import sys
import threading
import time
import urllib.request

DEFAULT_API = "http://127.0.0.1:8080/api/detections"


def _read_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except Exception:
        return None


def _read_det(api):
    try:
        with urllib.request.urlopen(api, timeout=0.5) as r:
            return json.loads(r.read())
    except Exception:
        return None


def record(path, stop_event, rate=10.0, api=DEFAULT_API, on_sample=None, log=print):
    """采样循环。stop_event.set() 即停。返回 summary dict。"""
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    period = 1.0 / max(1.0, float(rate))
    t0 = time.monotonic()
    n = 0
    ok_n = 0
    last_print = 0.0
    log(f"[rec] 开始 → {path} (rate={rate}Hz)")
    with open(path, "w", encoding="utf-8") as f:
        while not stop_event.is_set():
            t = time.monotonic()
            o = _read_json("/dev/shm/owner.json")
            o_age = None
            o_ok = False
            o_box = None
            o_cx = None
            if o:
                o_age = round(time.time() - float(o.get("ts", 0)), 3)
                o_ok = o_age <= 0.5
                o_box = o.get("bbox")
                if o_box:
                    o_cx = round((float(o_box[0]) + float(o_box[2])) / 2.0, 1)
            d = _read_det(api)
            persons = []
            fps = age_ms = None
            if d:
                fps, age_ms = d.get("fps"), d.get("age_ms")
                for x in (d.get("detections") or []):
                    if x.get("c") == "person":
                        persons.append([x.get("x1"), x.get("y1"), x.get("x2"), x.get("y2"),
                                        x.get("p")])
            imu = _read_json("/dev/shm/imu.json")
            hs = _read_json("/dev/shm/hri_state.json")
            fs = _read_json("/dev/shm/follow_status.json")
            fs_fresh = bool(fs and (time.monotonic() - float(fs.get("t", 0)) < 1.0))
            rec = {"t": round(t, 4), "ts": round(time.time(), 3),
                   "owner_ok": bool(o_ok), "owner_age": o_age,
                   "owner_box": o_box, "owner_cx": o_cx,
                   "n_person": len(persons), "persons": persons,
                   "fps": fps, "det_age_ms": age_ms,
                   "yaw": None if not imu else imu.get("yaw_deg"),
                   "yaw_rate": None if not imu else imu.get("yaw_rate_dps"),
                   "hri_state": None if not hs else hs.get("state"),
                   "hri_action": None if not hs else hs.get("action"),
                   "parked": None if not hs else hs.get("parked"),
                   "follow_fresh": fs_fresh,
                   "follow_state": None if not fs_fresh else fs.get("state"),
                   "follow_cmd": None if not fs_fresh else fs.get("cmd")}
            try:
                f.write(json.dumps(rec, ensure_ascii=False) + "\n")
                f.flush()
            except Exception as e:
                log(f"[rec] write error: {e}")
                break
            n += 1
            ok_n += 1 if o_ok else 0
            if on_sample is not None:
                try:
                    on_sample(rec, n, ok_n)
                except Exception:
                    pass
            now = time.monotonic()
            if now - last_print >= 1.0:
                last_print = now
                log(f"[rec] {now - t0:5.1f}s owner={'OK' if o_ok else '--'}"
                    f" n_person={len(persons)} fps={fps} yaw={rec['yaw']}"
                    f" hri={rec['hri_state']}/{rec['hri_action']}")
            dt = period - (time.monotonic() - t)
            if dt > 0:
                stop_event.wait(dt)
    dur = time.monotonic() - t0
    vis = (100.0 * ok_n / n) if n else 0.0
    log(f"[rec] 结束: {n} 帧 / {dur:.1f}s, owner 锁定率 {vis:.1f}%")
    return {"path": path, "n": n, "dur": round(dur, 1), "vis": round(vis, 1)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=None)
    ap.add_argument("--secs", type=float, default=0.0, help="0=Ctrl-C 停止")
    ap.add_argument("--rate", type=float, default=10.0)
    ap.add_argument("--api", default=DEFAULT_API)
    args = ap.parse_args()
    out = args.out or os.path.join(os.path.expanduser("~/hri_logs"),
                                   time.strftime("passby_%Y%m%d_%H%M%S.jsonl"))
    stop = threading.Event()
    try:
        signal.signal(signal.SIGINT, lambda *a: stop.set())
        signal.signal(signal.SIGTERM, lambda *a: stop.set())
    except Exception:
        pass
    if args.secs:
        threading.Timer(args.secs, stop.set).start()
    s = record(out, stop, rate=args.rate, api=args.api)
    print(f"[rec] 分析: python3 {os.path.join(os.path.dirname(os.path.abspath(__file__)), 'passby_analyze.py')} {s['path']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
