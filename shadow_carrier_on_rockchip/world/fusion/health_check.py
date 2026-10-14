#!/usr/bin/env python3
"""health_check.py —— 跟人链路健康监测（2026-10-14 统筹派单 @世界）

监测项（全部只读，不改任何服务）:
  - /api/detections : age_ms / fps / frames / stats(read_fail, restarts, infer_ms)
  - /dev/shm/imu.json 新鲜度（age）
  - /dev/shm/world_map.json、grid.json 新鲜度（[世界] 产物）
阈值（可用 --age/--fps 覆盖）:
  age_ms > 2000 → CRIT（跟随/凝视会瞎）    fps < 8 → WARN
  restarts 增长 → CRIT（daemon 不稳）      infer_ms > 150 → WARN
  imu age > 0.5s → WARN                    grid age > 5s → WARN
用法:
  python3 health_check.py            # 单次体检
  python3 health_check.py --watch 10 # 连续 10 轮(1Hz) 观察 restarts/fps 趋势
  python3 health_check.py --json     # 机器可读
退出码: 0=OK  1=WARN  2=CRIT
"""
import argparse
import json
import os
import sys
import time
import urllib.request

API = "http://127.0.0.1:8080/api/detections"
IMU = "/dev/shm/imu.json"
MAPS = ["/dev/shm/world_map.json", "/home/kickpi/world_lab/fusion/grid.json"]


def _get(url, timeout=3.0):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return json.loads(r.read().decode())


def _age_s(path):
    try:
        return time.time() - os.path.getmtime(path)
    except OSError:
        return None


def once(args, prev_restarts):
    row = {"t": time.strftime("%H:%M:%S"), "level": "OK", "notes": []}
    try:
        d = _get(API)
    except Exception as e:
        row["level"] = "CRIT"
        row["notes"].append("api不可达:%s" % e)
        return row, prev_restarts
    age = d.get("age_ms")
    fps = d.get("fps")
    st = d.get("stats", {})
    restarts = st.get("restarts")
    infer = st.get("infer_ms")
    n = len(d.get("detections", []))
    row.update({"age_ms": age, "fps": fps, "n": n, "infer_ms": infer, "restarts": restarts})
    if age is None:
        row["level"] = "WARN"
        row["notes"].append("无age字段(旧版?)")
    elif age > args.age:
        row["level"] = "CRIT"
        row["notes"].append("检测age %.0fms>%d" % (age, args.age))
    if fps is not None and fps < args.fps:
        if row["level"] == "OK":
            row["level"] = "WARN"
        row["notes"].append("fps %.1f<%d" % (fps, args.fps))
    if infer is not None and infer > 150:
        if row["level"] == "OK":
            row["level"] = "WARN"
        row["notes"].append("infer %.0fms" % infer)
    if prev_restarts is not None and restarts is not None and restarts > prev_restarts:
        row["level"] = "CRIT"
        row["notes"].append("daemon重启 +%d" % (restarts - prev_restarts))
    ia = _age_s(IMU)
    if ia is not None:
        row["imu_age_s"] = round(ia, 2)
        if ia > 0.5 and row["level"] == "OK":
            row["level"] = "WARN"
            row["notes"].append("imu age %.2fs" % ia)
    for m in MAPS:
        a = _age_s(m)
        if a is not None:
            row[os.path.basename(m) + "_age_s"] = round(a, 1)
            if a > 5.0 and row["level"] == "OK":
                row["level"] = "WARN"
                row["notes"].append("%s age %.0fs" % (os.path.basename(m), a))
    return row, (restarts if restarts is not None else prev_restarts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--watch", type=int, default=0, help="轮数, 0=单次")
    ap.add_argument("--age", type=int, default=2000, help="检测 age 阈值 ms")
    ap.add_argument("--fps", type=float, default=8.0, help="fps 下限")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    rounds = max(1, args.watch)
    prev = None
    worst, rows = "OK", []
    for i in range(rounds):
        row, prev = once(args, prev)
        rows.append(row)
        if row["level"] == "CRIT":
            worst = "CRIT"
        elif row["level"] == "WARN" and worst == "OK":
            worst = "WARN"
        if not args.json and (rounds == 1 or row["level"] != "OK" or i == rounds - 1):
            print("[%s] %-4s age=%s fps=%s n=%s infer=%s restarts=%s %s" % (
                row["t"], row["level"], row.get("age_ms"), row.get("fps"), row.get("n"),
                row.get("infer_ms"), row.get("restarts"),
                ("| " + "; ".join(row["notes"])) if row["notes"] else ""))
        if i < rounds - 1:
            time.sleep(1.0)
    if args.json:
        print(json.dumps({"level": worst, "rows": rows}, ensure_ascii=False))
    sys.exit({"OK": 0, "WARN": 1, "CRIT": 2}[worst])


if __name__ == "__main__":
    main()
