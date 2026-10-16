#!/usr/bin/env python3
"""鲁棒性报告: 解析 sim_session jsonl → 每场景判定 + 汇总表。
用法: python3 analyze_robustness.py logs/rob_<scenario>_run<k>.jsonl ..."""
import json
import sys
from collections import Counter
from pathlib import Path

HIDE_SPOT = (1.0, -4.3)
LIDAR_TEST_T = (6.0, 26.0)   # lidar_sweep_block 的注入窗口(与控制器 _scenario_post 对齐)


def analyze(path):
    recs, bad = [], 0
    for line in open(path, encoding="utf-8"):
        line = line.strip()
        if not line:
            continue
        try:
            recs.append(json.loads(line))
        except json.JSONDecodeError:
            bad += 1          # 被 kill 的运行会在末尾留半行 → 跳过, 不让整份报告崩掉
    scenario = next((r.get("scenario", "baseline") for r in recs
                     if r["type"] == "session_start"), "baseline")
    frames = [r for r in recs if r["type"] == "frame"]
    transitions = [r.get("msg", "") for r in recs if r["type"] == "transition"]
    out = {"scenario": scenario, "log": str(path), "truncated_lines": bad}

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

    # ---- passby: 底盘转向通道(原地转/掉头搜索)验收 (2026-10-16) ----
    # 窗口 [6,12]: 主人从车正前走到正后(最近 1.0m 擦身)。云台冻结 + 水平 FOV ±32°,
    # 只有底盘自转能守住视野 → 这里量化"守住没守住"。
    out["turn_on"] = bool(next((r.get("turn_on") for r in recs
                                if r["type"] == "session_start"), False))
    out["gt_turn"] = bool(next((r.get("gt_turn") for r in recs
                                if r["type"] == "session_start"), True))
    win = [f for f in frames if 6.0 <= f.get("t", -1) <= 12.0]
    ph = Counter((f.get("follow") or {}).get("phase") for f in win if f.get("follow"))
    out["follow_phases"] = {k: v for k, v in ph.items() if k}
    out["passby_turned_frames"] = sum(v for k, v in ph.items()
                                      if k in ("TURN", "BLIND", "SEARCH"))
    dirs = [d for d in ((f.get("follow") or {}).get("turn_dir") for f in win
                        if f.get("follow")) if d]
    out["passby_dir_flips"] = sum(1 for a, b in zip(dirs, dirs[1:]) if a != b)
    run = best = 0
    for f in win:
        run = 0 if "person" in (f.get("dets") or []) else run + 1
        best = max(best, run)
    step = (win[1]["t"] - win[0]["t"]) if len(win) > 1 else 0.3
    out["passby_invisible_max_s"] = round(best * max(step, 0.01), 2)
    out["passby_owner_visible_ratio"] = (
        round(sum(1 for f in win if "person" in (f.get("dets") or [])) / len(win), 2)
        if win else None)
    out["passby_min_dist_m"] = round(min(
        [((f["owner"][0] - f["robot"][0]) ** 2 + (f["owner"][2] - f["robot"][2]) ** 2) ** 0.5
         for f in win if f.get("owner") and f.get("robot")] or [99.0]), 2)

    # V2 雷达(SIM_LIDAR=1 才有字段): 受阻/否决/过滤次数 + veto 期间转没转
    out["lidar_on"] = bool(next((r.get("lidar") for r in recs
                                 if r["type"] == "session_start"), False))
    blocked_of = lambda f: set((f.get("lidar") or {}).get("blocked") or [])
    out["lidar_blocked_frames"] = sum(1 for f in frames if blocked_of(f))
    out["lidar_rot_blocked_frames"] = sum(1 for f in frames if "rot" in blocked_of(f))
    out["lidar_filters"] = sum(1 for r in recs if r["type"] == "lidar_filter")
    veto_yaws = [f["yaw"] for f in frames if "rot" in blocked_of(f) and "yaw" in f]
    out["lidar_veto_yaw_span"] = (round(max(veto_yaws) - min(veto_yaws), 3)
                                  if veto_yaws else None)
    # 只看注入窗口: 窗口外的 veto 是伴生现象(主人/路人走近也要禁转), 不是本场景要证的命题
    win = [f for f in frames if LIDAR_TEST_T[0] <= f["t"] <= LIDAR_TEST_T[1]]
    win_veto = [f for f in win if "rot" in blocked_of(f)]
    out["lidar_win_frames"] = len(win)
    out["lidar_veto_win_frames"] = len(win_veto)
    out["lidar_win_yaw_span"] = (round(max(f["yaw"] for f in win_veto)
                                       - min(f["yaw"] for f in win_veto), 3)
                                 if win_veto else None)
    out["sonar_blocked_in_window"] = sum(1 for f in win if f.get("sonar_cm", 200) < 20)

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
    if scenario not in ("bench_block", "occlude", "lidar_sweep_block",
                        "passby") and not got_receive:
        verdict = "FAIL"
        notes.append("缺 RECEIVE 转移")
    if scenario == "bench_block" and out["sonar_blocked_frames"] < 3:
        verdict = "FAIL"
        notes.append("sonar 受阻未触发")
    if scenario == "lidar_sweep_block":
        # 互补矩阵的受控证据: 注入窗口内超声(前向锥)全程 CLEAR, 雷达(360°)否决转向, 且车真没转
        if not out["lidar_on"]:
            verdict = "FAIL"
            notes.append("雷达未开(SIM_LIDAR=1 未传)")
        elif out["lidar_veto_win_frames"] < 3:
            verdict = "FAIL"
            notes.append(f"窗口内旋转净空未触发({out['lidar_veto_win_frames']} 帧)")
        if out["sonar_blocked_in_window"] > 0:
            verdict = "FAIL"
            notes.append(f"窗口内超声被误触发 {out['sonar_blocked_in_window']} 帧(障碍应在其盲区)")
        span = out.get("lidar_win_yaw_span")
        if span is None or abs(span) > 0.15:
            verdict = "FAIL"
            notes.append(f"veto 期间车仍转了 {span} rad")
        else:
            notes.append(f"窗口内: 超声全程 CLEAR({out['lidar_win_frames']} 帧) + 雷达 veto "
                         f"{out['lidar_veto_win_frames']} 帧, 车只转 {span} rad, 过滤 "
                         f"{out['lidar_filters']} 次")
    if scenario == "passby":
        # A/B: SIM_TURN=1(转向通道开) 应守住视野; SIM_TURN=0 是"原地转没接线"的对照基线,
        # 不判 FAIL(它是拿来做归因的负样本), 但必须关掉 ground-truth 假动作才算数。
        if out["gt_turn"]:
            verdict = "FAIL"
            notes.append("SIM_GT_TURN 未关 → 掉头是 ground-truth 假动作, 验收无效")
        elif not out["turn_on"]:
            verdict = "BASE"
            notes.append(f"对照组(turn off): 最长不可见 "
                         f"{out['passby_invisible_max_s']}s, 可见率 "
                         f"{out['passby_owner_visible_ratio']}, 转向 "
                         f"{out['passby_turned_frames']} 帧")
        elif out["passby_turned_frames"] < 3:
            verdict = "FAIL"
            notes.append(f"转向通道未启动 (phase 计数 {out['follow_phases']})")
        elif out["passby_invisible_max_s"] > 2.5:
            verdict = "FAIL"
            notes.append(f"擦身期间视野未守住: 最长不可见 "
                         f"{out['passby_invisible_max_s']}s (可见率 "
                         f"{out['passby_owner_visible_ratio']})")
        elif out["passby_dir_flips"] > 3:
            verdict = "FAIL"
            notes.append(f"转向方向抖动 {out['passby_dir_flips']} 次")
        else:
            notes.append(f"擦身 {out['passby_min_dist_m']}m: 转向 "
                         f"{out['passby_turned_frames']} 帧({out['follow_phases']}), "
                         f"最长不可见 {out['passby_invisible_max_s']}s, 反转 "
                         f"{out['passby_dir_flips']} 次")
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
