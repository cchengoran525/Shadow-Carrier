#!/usr/bin/env python3
"""passby_record.py — T4 passby 客观记录器 (10Hz 采样 → JSONL)

用途: passby 时同步记录"主人是否被锁/框在哪/几个人/IMU 航向/HRI 状态",
      跑完交给 passby_analyze.py 自动算 可见率 / 最长缺失 / 疑似换主。

用法(板端):
  python3 passby_record.py                       # 自动命名 ~/hri_logs/passby_<时间>.jsonl
  python3 passby_record.py --secs 40             # 录 40s 自动停
  python3 passby_record.py --out /tmp/p1.jsonl   # 指定输出
  # 录完 Ctrl-C 即停; 然后: python3 passby_analyze.py <上面打印的路径>

只读: owner.json / /api/detections / imu.json / hri_state.json —— 不控车、不改任何服务。
"""
import argparse
import json
import os
import sys
import time
import urllib.request


def _read_json(path, max_age=None):
    try:
        with open(path) as f:
            d = json.load(f)
        if max_age is not None and time.time() - float(d.get("ts", 0)) > max_age:
            return None
        return d
    except Exception:
        return None


def _det(api):
    try:
        with urllib.request.urlopen(api, timeout=0.5) as r:
            return json.loads(r.read())
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=None)
    ap.add_argument("--secs", type=float, default=0.0, help="0=Ctrl-C 停止")
    ap.add_argument("--rate", type=float, default=10.0)
    ap.add_argument("--api", default="http://127.0.0.1:8080/api/detections")
    args = ap.parse_args()

    out = args.out or os.path.join(
        os.path.expanduser("~/hri_logs"),
        time.strftime("passby_%Y%m%d_%H%M%S.jsonl"))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    period = 1.0 / max(1.0, args.rate)
    t0 = time.monotonic()
    n = 0
    last_print = 0.0
    owner_ok_n = 0
    print(f"[rec] 开始 → {out}  (rate={args.rate}Hz, {('%.0fs' % args.secs) if args.secs else 'Ctrl-C 停止'})")
    try:
        with open(out, "w", encoding="utf-8") as f:
            while not args.secs or (time.monotonic() - t0) < args.secs:
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
                d = _det(args.api)
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
                rec = {
                    "t": round(t, 4), "ts": round(time.time(), 3),
                    "owner_ok": bool(o_ok), "owner_age": o_age,
                    "owner_box": o_box, "owner_cx": o_cx,
                    "n_person": len(persons), "persons": persons,
                    "fps": fps, "det_age_ms": age_ms,
                    "yaw": None if not imu else imu.get("yaw_deg"),
                    "yaw_rate": None if not imu else imu.get("yaw_rate_dps"),
                    "hri_state": None if not hs else hs.get("state"),
                    "hri_action": None if not hs else hs.get("action"),
                    "parked": None if not hs else hs.get("parked"),
                }
                f.write(json.dumps(rec, ensure_ascii=False) + "\n")
                f.flush()
                n += 1
                owner_ok_n += 1 if o_ok else 0
                now = time.monotonic()
                if now - last_print >= 1.0:
                    last_print = now
                    print(f"[rec] {now - t0:5.1f}s  owner={'OK' if o_ok else '--'}"
                          f" n_person={len(persons)} fps={fps} yaw={rec['yaw']}"
                          f" hri={rec['hri_state']}/{rec['hri_action']}")
                dt = period - (time.monotonic() - t)
                if dt > 0:
                    time.sleep(dt)
    except KeyboardInterrupt:
        pass
    dur = time.monotonic() - t0
    vis = (100.0 * owner_ok_n / n) if n else 0.0
    print(f"\n[rec] 结束: {n} 帧 / {dur:.1f}s, owner 锁定率 {vis:.1f}%")
    print(f"[rec] 分析: python3 {os.path.join(os.path.dirname(os.path.abspath(__file__)), 'passby_analyze.py')} {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
