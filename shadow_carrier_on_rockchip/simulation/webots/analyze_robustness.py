#!/usr/bin/env python3
"""鲁棒性报告: 解析 sim_session jsonl → 每场景判定 + 汇总表。
用法: python3 analyze_robustness.py logs/rob_<scenario>_run<k>.jsonl ..."""
import json
import sys
from collections import Counter
from pathlib import Path

HIDE_SPOT = (1.0, -4.3)


def analyze(path):
    recs = [json.loads(l) for l in open(path, encoding="utf-8")]
    scenario = next((r.get("scenario", "baseline") for r in recs
                     if r["type"] == "session_start"), "baseline")
    frames = [r for r in recs if r["type"] == "frame"]
    transitions = [r.get("msg", "") for r in recs if r["type"] == "transition"]
    out = {"scenario": scenario, "log": str(path)}

    out["completed"] = any(r["type"] == "session_end"
                           and r.get("reason") == "autotest_done" for r in recs)
    out["duration_s"] = recs[-1].get("t", 0)
    states = [f["state"] for f in frames]
    out["states"] = dict(Counter(states))
    seq = []
    for s in states:
        if not seq or seq[-1] != s:
            seq.append(s)
    out["state_seq"] = ">".join(seq)
    got_wait = any("-> WAIT" in m for m in transitions)
    got_hide = any("-> HIDE" in m for m in transitions)
    got_receive = any("-> RECEIVE" in m for m in transitions)
    out["transitions"] = transitions

    # 安全: 越出走廊 / 撞人距离
    out["wall_violations"] = sum(1 for f in frames
                                 if abs(f["robot"][0]) > 1.32 or abs(f["robot"][2]) > 5.95)
    dists = [((f["owner"][0] - f["robot"][0]) ** 2 + (f["owner"][2] - f["robot"][2]) ** 2) ** 0.5
             for f in frames if f.get("owner") and f.get("robot")]
    out["min_owner_dist_m"] = round(min(dists), 2) if dists else None

    # sonar 受阻
    out["sonar_blocked_frames"] = sum(1 for f in frames if f.get("sonar_cm", 200) < 20)

    # HIDE 驶停与最终停靠点(用 HIDE 状态最后一帧的机器人位置)
    hides = [r for r in recs if r["type"] == "hide_drive"]
    out["hide_parked"] = bool(hides) and hides[-1]["remaining"] <= 0.55
    hide_frames = [f for f in frames if f["state"] == "HIDE"]
    if hide_frames:
        rp = hide_frames[-1]["robot"]
        out["hide_park_dist_from_hidespot"] = round(
            ((rp[0] - HIDE_SPOT[0]) ** 2 + (rp[2] - HIDE_SPOT[1]) ** 2) ** 0.5, 2)

    # 主人不可见窗口(排除剧本末段主人远去)
    invis = [f for f in frames if not f["dets"] and f.get("owner")
             and ((f["owner"][0] - f["robot"][0]) ** 2 +
                  (f["owner"][2] - f["robot"][2]) ** 2) ** 0.5 < 6.0]
    out["owner_invisible_frames"] = len(invis)

    # 幻影注入统计
    out["phantom_adoption_risks"] = sum(1 for r in recs
                                        if r["type"] == "phantom_adoption_risk")
    out["phantom_frames"] = sum(1 for f in frames if f.get("phantoms"))

    notes = []
    verdict = "PASS"
    if not out["completed"]:
        verdict, _ = "FAIL", notes.append("未跑完(session_end 缺失)")
    if out["wall_violations"]:
        verdict = "FAIL"
        notes.append(f"越界 {out['wall_violations']} 帧")
    if scenario in ("baseline", "cross2", "occlude", "bench_block", "handoff_cross",
                    "phantom"):
        if not (got_wait and got_hide):
            verdict = "FAIL"
            notes.append("缺 WAIT/HIDE 转移")
    if scenario not in ("bench_block", "occlude") and not got_receive:
        verdict = "FAIL"
        notes.append("缺 RECEIVE 转移")
    if scenario == "bench_block" and out["sonar_blocked_frames"] < 3:
        verdict = "FAIL"
        notes.append("sonar 受阻未触发")
    if scenario == "hide_occupied":
        d = out.get("hide_park_dist_from_hidespot")
        if d is not None and d < 0.6:
            verdict = "FAIL"
            notes.append(f"HIDE 仍去被占点(距 HIDE_SPOT {d}m) → 暴露 person 打分缺口")
        else:
            notes.append(f"回退生效, 停靠点距被占点 {d}m")
    if scenario == "handoff_cross" and out["owner_invisible_frames"] > 0 and got_receive:
        notes.append("交接中遮挡后仍完成 RECEIVE")
    if scenario == "phantom":
        n = out["phantom_adoption_risks"]
        notes.append(f"幻影帧 {out['phantom_frames']}, 认主回退暴露 {n} 拍"
                     + ("(链路仍完成)" if got_receive else ""))
    out["notes"] = notes
    out["verdict"] = verdict
    return out


def main(paths):
    results = [analyze(p) for p in paths]
    for r in results:
        print(f"[{r['verdict']:>4}] {Path(r['log']).name}  scenario={r['scenario']} "
              f"dur={r['duration_s']}s states={r['states']} "
              f"wall_viol={r['wall_violations']} min_dist={r['min_owner_dist_m']} "
              f"hide_park={r.get('hide_park_dist_from_hidespot')}m")
        if r["notes"]:
            print(f"        notes: {'; '.join(r['notes'])}")
        for m in r["transitions"]:
            print(f"        {m[:100]}")
    n_pass = sum(1 for r in results if r["verdict"] == "PASS")
    print(f"\n=== {n_pass}/{len(results)} PASS ===")
    return 0 if n_pass == len(results) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
