#!/usr/bin/env python3
"""passby_analyze.py — 读 passby_record.py 的 JSONL, 输出 T4 的客观指标

  python3 passby_analyze.py ~/hri_logs/passby_xxx.jsonl
  python3 passby_analyze.py <file> --vis-ok 90 --gap-ok 0.5 --switch-px 200

能自动算的:
  · 主人锁定率(owner.json 新鲜占比) —— 身份锁定的严格口径
  · 画面有人率(person 检测占比)   —— 宽松口径(含路人)
  · 最长连续缺失 / 超过阈值的缺失段
  · 疑似换主(锁定连续段内中心跳变 / 缺失后从另一侧重现)
  · 朝向一致性(人偏一侧时, IMU yaw 是否朝该侧转; 粗检, 仅作参考)
不能自动算(按 FIELD_TEST_PLAN: 视频+卷尺):
  · "停住≤1.0s" 与 "横向净空≥0.6m" —— 请用视频/卷尺判定后手填 §6 表。

退出码: 0=客观项全过 / 1=有未过 / 2=输入不足
"""
import argparse
import json
import math
import sys


def wrap180(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def load(path):
    recs = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                recs.append(json.loads(line))
            except Exception:
                pass
    recs.sort(key=lambda r: r.get("t", 0.0))
    return recs


def bearing_of(cx):
    return math.degrees(math.atan((float(cx) - 320.0) / 508.2))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file")
    ap.add_argument("--vis-ok", type=float, default=90.0, help="owner 锁定率下限 %%")
    ap.add_argument("--gap-ok", type=float, default=0.5, help="最长连续缺失上限 s")
    ap.add_argument("--switch-px", type=float, default=200.0, help="判定换主的中心跳变 px")
    ap.add_argument("--skip", type=float, default=1.5,
                    help="跳过开头启动段(秒); 0=不跳. 原始/稳态锁定率都会打印")
    args = ap.parse_args()

    all_recs = load(args.file)
    if len(all_recs) < 10:
        print(f"[analyze] 样本太少({len(all_recs)}), 无法判定"); return 2
    t0_all = all_recs[0]["t"]
    dur_all = all_recs[-1]["t"] - t0_all
    raw_n = len(all_recs)
    raw_ok = sum(1 for r in all_recs if r.get("owner_ok"))
    raw_vis = 100.0 * raw_ok / raw_n if raw_n else 0.0

    recs = all_recs
    if args.skip > 0:
        recs = [r for r in all_recs if r["t"] - t0_all >= args.skip]
        if len(recs) < 10:          # 太短就不跳
            recs = all_recs
    if len(recs) < 10:
        print(f"[analyze] 样本太少({len(recs)}), 无法判定"); return 2

    t0 = t0_all
    t1 = recs[-1]["t"]
    dur = max(1e-6, t1 - recs[0]["t"])
    n = len(recs)
    n_ok = sum(1 for r in recs if r.get("owner_ok"))
    n_person = sum(1 for r in recs if (r.get("n_person") or 0) > 0)
    vis = 100.0 * n_ok / n
    person_pct = 100.0 * n_person / n

    # ---- 连续缺失段 ----
    gaps = []          # (start_t, end_t, seconds)
    g_start = None
    prev_t = None
    for r in recs:
        t = r["t"]
        if r.get("owner_ok"):
            if g_start is not None:
                gaps.append((g_start, prev_t, prev_t - g_start))
                g_start = None
        else:
            if g_start is None:
                g_start = t
            prev_t = t
    if g_start is not None:
        gaps.append((g_start, prev_t, prev_t - g_start))
    max_gap = max([g[2] for g in gaps], default=0.0)
    bad_gaps = [g for g in gaps if g[2] > args.gap_ok]

    # ---- 命令停止段(有新鲜 follow_status 时才算; 判"停住≤1.0s") ----
    has_cmd = any(r.get("follow_fresh") for r in recs)
    stops = []
    if has_cmd:
        s_start = None
        prev_t2 = None
        for r in recs:
            if not r.get("follow_fresh"):
                if s_start is not None:
                    stops.append((s_start, prev_t2, prev_t2 - s_start))
                    s_start = None
                continue
            t = r["t"]
            moving = r.get("follow_cmd") not in (None, "STOP")
            if not moving:
                if s_start is None:
                    s_start = t
                prev_t2 = t
            elif s_start is not None:
                stops.append((s_start, prev_t2, prev_t2 - s_start))
                s_start = None
        if s_start is not None:
            stops.append((s_start, prev_t2, prev_t2 - s_start))
    long_stops = [s for s in stops if s[2] >= 0.3]
    max_stop = max([s[2] for s in long_stops], default=0.0)
    bad_stops = [s for s in long_stops if s[2] > 1.0]

    # ---- 疑似换主 ----
    # (a) 锁定连续段内, 相邻采样(Δt<=0.4s)中心跳变 > switch_px
    # (b) 一段缺失后重新锁定, 中心相对缺失前跳到另一侧(> switch_px)
    switches = []
    prev_ok_cx = None
    last_before_gap = None
    for i, r in enumerate(recs):
        if not r.get("owner_ok"):
            if prev_ok_cx is not None:
                last_before_gap = prev_ok_cx
            prev_ok_cx = None
            continue
        cx = r.get("owner_cx")
        if cx is None:
            continue
        if prev_ok_cx is not None:
            dt = r["t"] - recs[i - 1]["t"]
            if dt <= 0.4 and abs(cx - prev_ok_cx) > args.switch_px:
                switches.append((r["t"] - t0, "jump", round(prev_ok_cx, 0), round(cx, 0)))
        elif last_before_gap is not None and abs(cx - last_before_gap) > args.switch_px:
            switches.append((r["t"] - t0, "reappear", round(last_before_gap, 0), round(cx, 0)))
            last_before_gap = None
        prev_ok_cx = cx

    # ---- 朝向一致性(粗检) ----
    # 人偏右(bearing>0) → 车应右转 → IMU yaw 减小(MOVE R); 反之增大。
    consistent = judged = 0
    for i, r in enumerate(recs):
        if not r.get("owner_ok") or r.get("owner_cx") is None:
            continue
        b = bearing_of(r["owner_cx"])
        if abs(b) < 20.0:
            continue
        y0 = r.get("yaw")
        y1 = None
        for j in range(i + 1, min(i + 6, len(recs))):   # ~0.4s 后
            if recs[j].get("yaw") is not None:
                y1 = recs[j]["yaw"]; break
        if y0 is None or y1 is None:
            continue
        dy = wrap180(y1 - y0)
        if abs(dy) < 1.0:      # 几乎没转, 不计
            continue
        judged += 1
        # bearing>0(右) 期望 dy<0; bearing<0 期望 dy>0
        if (b > 0 and dy < 0) or (b < 0 and dy > 0):
            consistent += 1
    dir_pct = (100.0 * consistent / judged) if judged else None

    # ---- HRI 动作时间线(T9/T10: 只展示, 不改行为) ----
    timeline = []
    prev = None
    for r in recs:
        key = (r.get("hri_state"), r.get("hri_action"), bool(r.get("parked")))
        if key != prev:
            timeline.append((round(r["t"] - t0, 1),) + key)
            prev = key

    # ---- 打印 ----
    print(f"== passby 分析: {args.file}")
    if recs[0].get("label"):
        print(f"   场景标签         : {recs[0].get('label')}")
        if recs[0].get("label") != "T4":
            print("   注意: 当前自动阈值按 T4 设定; 本场景请按 FIELD_TEST_PLAN 对应判据解读")
    print(f"   时长 {dur_all:.1f}s (稳态 {dur:.1f}s, 跳过前 {args.skip:.1f}s) / 样本 {n} / 采样率 {n / dur:.1f}Hz")
    print(f"   主人锁定率(严格) : {vis:5.1f}%  (原始 {raw_vis:.1f}%)  [阈值 ≥{args.vis_ok:.0f}%]")
    print(f"   画面有人率(宽松) : {person_pct:5.1f}%")
    print(f"   最长连续缺失     : {max_gap:.2f}s   [阈值 ≤{args.gap_ok:.2f}s]"
          f"   超阈段 {len(bad_gaps)} 个")
    if gaps:
        print("   缺失段: " + ", ".join(f"{g[0]-t0:.1f}s/{g[2]:.2f}s" for g in gaps[:12])
              + (" ..." if len(gaps) > 12 else ""))
    print(f"   疑似换主         : {len(switches)} 次   [阈值 0]")
    if has_cmd:
        print(f"   命令停止段(≥0.3s): {len(long_stops):>3} 段, 最长 {max_stop:.2f}s   [阈值 ≤1.0s]")
    else:
        print("   命令停止段       : 无 follow_status(旧录制/手动), 需视频手判")
    for s in switches[:10]:
        print(f"      @{s[0]:.1f}s {s[1]} {s[2]}→{s[3]}px")
    if dir_pct is None:
        print("   朝向一致性(粗检) : 样本不足(人需明显偏一侧且车在转)")
    else:
        print(f"   朝向一致性(粗检) : {dir_pct:5.1f}% ({consistent}/{judged})  "
              f"[参考; 人也在动, 不等于控制方向正确]")
    if timeline:
        tl = " → ".join(
            f"{t}s:{s or '-'}/{a or '-'}" + ("/P" if p else "") for t, s, a, p in timeline[:20])
        print(f"   HRI 时间线(T9/10): {tl}" + (" ..." if len(timeline) > 20 else ""))

    fails = []
    if vis < args.vis_ok:
        fails.append(f"锁定率 {vis:.1f}% < {args.vis_ok:.0f}%")
    if max_gap > args.gap_ok:
        fails.append(f"最长缺失 {max_gap:.2f}s > {args.gap_ok:.2f}s")
    if switches:
        fails.append(f"疑似换主 {len(switches)} 次")
    if has_cmd and max_stop > 1.0:
        fails.append(f"最长停止 {max_stop:.2f}s > 1.0s")
    print("\n   自动项: " + ("✅ PASS" if not fails else "❌ " + "; ".join(fails)))
    if has_cmd:
        print("   手填项: 横向净空(≥0.6m)、转向是否朝人 —— 看视频 + 卷尺（停住已自动判）")
    else:
        print("   手填项: 停住次数/最长停顿(≤1.0s)、横向净空(≥0.6m)、转向是否朝人 —— 看视频 + 卷尺")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
