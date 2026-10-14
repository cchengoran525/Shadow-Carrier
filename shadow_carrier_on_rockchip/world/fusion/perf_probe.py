#!/usr/bin/env python3
"""perf_probe.py —— RK3566 性能压力归因探针（NPU / CPU / 进程 / IO）

2026-10-14 [世界]：跟人事故后需要"到底卡在哪"的定量答案。本脚本只读，不改服务。
采样 N 秒，输出各层占用与**主导瓶颈判定**：
  - CPU : /proc/stat 全核 busy / iowait; /proc/<pid>/stat 各进程 CPU
  - NPU : /sys/kernel/debug/rknpu/load（需 root；不可读则标 N/A）
  - 流水线: /api/detections 的 fps / age_ms / infer_ms / restarts 趋势
  - IO  : /dev/shm 用量 + yolo_out.jpg 写入频率; iowait 占比
  - 热点: dnsmasq CPU 单列（历史两次跑飞 99.7%）
用法: python3 perf_probe.py [--secs 10] [--json]
退出码: 0=OK 1=WARN 2=CRIT（与 health_check 语义一致）
"""
import argparse
import json
import os
import time
import urllib.request

API = "http://127.0.0.1:8080/api/detections"
NPU_LOAD = "/sys/kernel/debug/rknpu/load"
SHM_JPG = "/dev/shm/yolo_out.jpg"
CLK = os.sysconf("SC_CLK_TCK")


def _read(path):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return None


def _cpu_times():
    """[(busy, total)] per core + aggregate iowait"""
    out = []
    txt = _read("/proc/stat")
    if not txt:
        return out
    for line in txt.splitlines():
        if not line.startswith("cpu"):
            continue
        parts = line.split()
        name = parts[0]
        vals = [int(x) for x in parts[1:]]
        idle = vals[3] + (vals[4] if len(vals) > 4 else 0)   # idle+iowait
        iowait = vals[4] if len(vals) > 4 else 0
        total = sum(vals)
        out.append((name, total - idle, total, iowait))
    return out


def _proc_cpu():
    """{pid: (comm, cpu_ticks)}"""
    out = {}
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        txt = _read("/proc/%s/stat" % pid)
        if not txt:
            continue
        try:
            rp = txt.rfind(")")
            comm = txt[txt.find("(") + 1:rp]
            parts = txt[rp + 2:].split()
            utime, stime = int(parts[11]), int(parts[12])
            out[int(pid)] = (comm, utime + stime)
        except Exception:
            continue
    return out


def _api():
    try:
        with urllib.request.urlopen(API, timeout=3) as r:
            return json.loads(r.read().decode())
    except Exception:
        return None


def _npu():
    txt = _read(NPU_LOAD)
    if txt is None:
        return None
    return txt.strip().replace("\n", " | ")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--secs", type=float, default=10.0)
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    c0 = {n: (b, t, io) for n, b, t, io in _cpu_times()}
    p0 = _proc_cpu()
    j0 = os.path.getmtime(SHM_JPG) if os.path.exists(SHM_JPG) else None
    a0 = _api()
    time.sleep(args.secs)
    c1 = {n: (b, t, io) for n, b, t, io in _cpu_times()}
    p1 = _proc_cpu()
    j1 = os.path.getmtime(SHM_JPG) if os.path.exists(SHM_JPG) else None
    a1 = _api()

    cores = [n for n in c1 if n != "cpu"]
    busy_pct, iowait_pct = [], []
    for n in cores:
        dt = c1[n][1] - c0.get(n, (0, 0, 0))[1]
        db = c1[n][0] - c0.get(n, (0, 0, 0))[0]
        dio = c1[n][2] - c0.get(n, (0, 0, 0))[2]
        if dt > 0:
            busy_pct.append(100.0 * db / dt)
            iowait_pct.append(100.0 * dio / dt)
    cpu_avg = round(sum(busy_pct) / len(busy_pct), 1) if busy_pct else None
    io_avg = round(sum(iowait_pct) / len(iowait_pct), 1) if iowait_pct else None

    procs = []
    for pid, (comm, t1) in p1.items():
        if pid in p0:
            d = (t1 - p0[pid][1]) / CLK / args.secs * 100.0
            if d > 1.0:
                procs.append((round(d, 1), comm, pid))
    procs.sort(reverse=True)

    dns = next((cpu for cpu, comm, _ in procs if comm == "dnsmasq"), 0.0)
    jpg_rate = None
    if j0 and j1:
        jpg_rate = round(max(0.0, (j1 - j0)) / args.secs, 1)

    res = {
        "secs": args.secs,
        "cpu_busy_avg_pct": cpu_avg,
        "cpu_busy_per_core_pct": [round(x, 1) for x in busy_pct],
        "iowait_avg_pct": io_avg,
        "top_procs": [{"cpu_pct": c, "comm": n, "pid": p} for c, n, p in procs[:6]],
        "dnsmasq_cpu_pct": dns,
        "npu_load": _npu(),
        "shm_jpg_writes_per_s": jpg_rate,
        "api": None,
    }
    notes, level = [], "OK"
    if a1:
        st = a1.get("stats", {})
        res["api"] = {"fps": a1.get("fps"), "age_ms": a1.get("age_ms"),
                      "infer_ms": st.get("infer_ms"), "restarts": st.get("restarts"),
                      "read_fail": st.get("read_fail")}
        if (a1.get("age_ms") or 0) > 2000:
            level = "CRIT"; notes.append("检测age %.0fms>2000（跟随会瞎）" % a1["age_ms"])
        if (a1.get("fps") or 99) < 8:
            notes.append("fps %.1f<8" % a1["fps"]); level = "CRIT" if level == "CRIT" else "WARN"
        if a0 and st.get("restarts", 0) > (a0.get("stats", {}) or {}).get("restarts", 0):
            level = "CRIT"; notes.append("daemon 在采样窗内又重启")
    else:
        level = "CRIT"; notes.append("api 不可达（流水线可能已死）")

    # 归因判定
    dom = "idle"
    if cpu_avg is not None and cpu_avg > 80:
        dom = "CPU"
    if io_avg is not None and io_avg > 20:
        dom = "IO"
    npu = res["npu_load"] or ""
    import re as _re
    npu_vals = [int(m) for m in _re.findall(r"(\d+)\s*%", npu)]
    npu_pct = max(npu_vals) if npu_vals else 0
    if npu_pct > 70:
        dom = "NPU"
    if dns and dns > 50:
        notes.append("dnsmasq %.0f%% CPU（历史跑飞根因）" % dns)
        dom = "CPU"
    res["dominant_bottleneck"] = dom
    res["notes"], res["level"] = notes, level

    if args.json:
        print(json.dumps(res, ensure_ascii=False, indent=1))
    else:
        print("[%s] 主导瓶颈=%s | CPU均%.1f%%(各核%s) iowait%.1f%% | NPU=%s | dnsmasq=%s%% | jpg写%.1f/s" % (
            time.strftime("%H:%M:%S"), dom, cpu_avg or -1, res["cpu_busy_per_core_pct"],
            io_avg if io_avg is not None else -1, npu or "N/A", dns, jpg_rate if jpg_rate is not None else -1))
        for c, n, p in procs[:6]:
            print("   %-14s %5.1f%%  pid=%d" % (n, c, p))
        if res["api"]:
            print("   api: fps=%s age=%sms infer=%sms restarts=%s read_fail=%s" % (
                res["api"]["fps"], res["api"]["age_ms"], res["api"]["infer_ms"],
                res["api"]["restarts"], res["api"]["read_fail"]))
        for n in notes:
            print("   ! %s" % n)
    import sys
    sys.exit({"OK": 0, "WARN": 1, "CRIT": 2}[level])


if __name__ == "__main__":
    main()
