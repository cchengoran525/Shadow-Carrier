#!/usr/bin/env python3
"""lidar_guard_daemon.py —— 扫描帧 → 安全包络 的常驻桥

数据流（三段解耦, 每段可单独替换/离线跑）:

    [扫描生产者]                [本 daemon]              [命令路径]
    YDLidar-SDK 桥 / 仿真    →  lidar_guard.update()  →  /dev/shm/lidar_safety.json
    写 /dev/shm/lidar_scan.json    (纯逻辑)              rk_control 的 uart_send 过滤

为什么用"一个文件"当扫描接口: 雷达 SDK 是 C++/需 aarch64 交叉编译, 与 RK 侧纯 Python
控制链路解耦后, 可以各自独立地坏、独立地重启; 也让 **没有雷达的今天** 就能用录制帧
/仿真帧把整条避障链路跑通验证（本文件 --replay/--once 就是为此准备的）。

用法:
  python3 lidar_guard_daemon.py                        # 常驻(轮询扫描文件)
  python3 lidar_guard_daemon.py --once                 # 读一帧 → 打印包络(装车自检)
  python3 lidar_guard_daemon.py --replay scans.jsonl --out /tmp/safety.jsonl
  python3 lidar_guard_daemon.py --record /data/scans.jsonl   # 顺带落原始帧(PC 科研轨/复现)
"""
import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lidar_guard as lg   # noqa: E402


def _read_json(path, retries=3):
    """容错读: 生产者可能正在替换文件 → 小退避重试"""
    for i in range(retries):
        try:
            with open(path, encoding="utf-8") as f:
                return json.load(f)
        except FileNotFoundError:
            return None
        except Exception:
            time.sleep(0.01 * (i + 1))
    return None


def _write_atomic(path, obj):
    tmp = f"{path}.tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False)
    os.replace(tmp, path)


def run_loop(args, guard):
    last_ts = None
    last_frame_id = None
    last_record = 0.0
    print(f"[guard] 轮询 {args.scan_file} → {args.out} @{args.rate}Hz "
          f"(config={args.config or '默认'})")
    while True:
        frame = _read_json(args.scan_file)
        if frame is not None:
            fid = (frame.get("frame_id"), frame.get("ts")) if isinstance(frame, dict) else None
            if fid != last_frame_id:
                last_frame_id = fid
                env = guard.update(frame)
                if args.out:
                    _write_atomic(args.out, env)
                if args.record and time.monotonic() - last_record >= 1.0 / args.record_rate:
                    last_record = time.monotonic()
                    with open(args.record, "a", encoding="utf-8") as f:
                        f.write(json.dumps(frame, ensure_ascii=False) + "\n")
                _report(env, args)
                last_ts = env["ts"]
        time.sleep(1.0 / args.rate)


_reported = {"mode": None, "blocked": None}


def _report(env, args):
    """节流打印: 只在 mode/blocked 变化或每 --verbose-s 秒打一行(日志不刷屏)"""
    if args.quiet:
        return
    blocked = {k: v for k, v in (env.get("blocked") or {}).items() if v}
    now = time.monotonic()
    changed = (env["mode"] != _reported["mode"]) or (blocked != _reported["blocked"])
    if changed or now - getattr(_report, "_t", 0.0) >= args.verbose_s:
        _report._t = now
        _reported["mode"], _reported["blocked"] = env["mode"], blocked
        near = env.get("nearest") or {}
        print(f"[guard] mode={env['mode']} ok={env['ok']} n={env['n_points']} "
              f"blocked={blocked or '-'} gap_fwd={env['gap_m'].get('fwd')} "
              f"vx_max={env['limits'].get('vx_max')} "
              f"nearest={near.get('cls')}@{near.get('bearing_deg')}°/{near.get('range_m')}m "
              f"legs={len(env.get('person_legs') or [])}"
              + (f" escape={env['escape']['dir']}" if env.get("escape") else ""))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scan-file", default="/dev/shm/lidar_scan.json",
                    help="扫描生产者写的最新帧文件")
    ap.add_argument("--out", default="/dev/shm/lidar_safety.json",
                    help="安全包络落盘路径(空 = 不落盘, 只打印)")
    ap.add_argument("--config", default=None, help="参数 JSON(默认 configs/lidar_guard.json)")
    ap.add_argument("--rate", type=float, default=20.0, help="轮询/发布频率 Hz")
    ap.add_argument("--once", action="store_true", help="读一帧, 打印包络后退出")
    ap.add_argument("--replay", default=None, help="回放扫描帧 JSONL(离线回归/调参)")
    ap.add_argument("--record", default=None, help="顺带把原始帧追加写入此 JSONL")
    ap.add_argument("--record-rate", type=float, default=2.0, help="录制频率 Hz(存盘友好)")
    ap.add_argument("--verbose-s", type=float, default=5.0, help="无变化时的打印间隔")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    cfg_path = args.config
    if cfg_path is None:
        default_cfg = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   "..", "..", "configs", "lidar_guard.json")
        cfg_path = default_cfg if os.path.exists(default_cfg) else None
    guard = lg.Guard(lg.load_config(cfg_path))
    print(f"[guard] 遮罩 {lg.mask_spans_deg(guard.cfg)} (铜柱 {len(guard.cfg['pillars_xy'])} 根)"
          f" 车身 {guard.cfg['body_half_w']*2:.2f}×{guard.cfg['body_half_l']*2:.2f}m"
          f" v_nom={guard.cfg['v_nom_mps']}m/s")

    if args.once:
        frame = _read_json(args.scan_file)
        if frame is None:
            print(f"[guard] 读不到 {args.scan_file} —— 扫描生产者还没起?")
            return 2
        env = guard.update(frame)
        print(json.dumps(env, ensure_ascii=False, indent=1))
        return 0

    if args.replay:
        out = []
        with open(args.replay, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                env = guard.update(json.loads(line))
                _report(env, args)
                out.append(env)
        if args.out and args.out.endswith(".jsonl"):
            with open(args.out, "w", encoding="utf-8") as f:
                for env in out:
                    f.write(json.dumps(env, ensure_ascii=False) + "\n")
            print(f"[guard] 回放 {len(out)} 帧 → {args.out}")
        else:
            print(f"[guard] 回放 {len(out)} 帧; 末帧 vx_max={out[-1]['limits']['vx_max']}")
        return 0

    try:
        run_loop(args, guard)
    except KeyboardInterrupt:
        print("\n[guard] 停止")
    return 0


if __name__ == "__main__":
    sys.exit(main())
