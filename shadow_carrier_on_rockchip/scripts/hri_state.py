#!/usr/bin/env python3
"""hri_state.py v0.2 - [HRI] 行为状态机
状态: FOLLOW / WAIT / HIDE / RECEIVE / YIELD
数据源: /api/detections + owner_score() [认主线冻结接口]
参数:   hri_params.json (物理量优先/像素降级/比值免疫, 换相机只改 camera 段)
门控:   自运动期间(车在动)不采信 bbox 位移证据
用法:
  真数据: python3 hri_state.py --live
  假数据: python3 hri_state.py --fake fake_script.jsonl
  换参数: python3 hri_state.py --live --params /path/hri_params.json
"""
import time, json, argparse, os, urllib.request

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PARAMS = os.path.join(SCRIPT_DIR, "hri_params.json")

DEFAULTS = {
    "camera": {"focal_px": 508.0, "person_height_m": 1.65,
               "frame_h": 480, "clip_ratio": 0.95, "shoulder_width_m": 0.45},
    "metric": {"static_disp_m": 0.08, "static_v_mps": 0.15,
               "wake_disp_m": 0.40, "near_dist_m": 1.2,
               "bend_near_dist_m": 2.5, "receive_exit_s": 0.8,
               "v_ema": 0.4, "v_max_mps": 2.5,
               "approach_fast_mps": 0.8, "approach_slow_mps": 0.15,
               "recede_mps": 0.05},
    "pixel": {"static_disp_px": 15, "wake_disp_px": 40, "near_h_px": 300,
              "approach_fast_rate": 1.2, "approach_slow_rate": 0.08},
    "ratios": {"bend": 0.25, "bend_clear": 0.10,
               "base_up_alpha": 0.15, "base_down_alpha": 0.005},
    "timing": {"static_need_s": 5.0, "dwell_s": 1.0, "ego_quiet_s": 1.0,
               "hide_after_s": 20.0, "receive_timeout_s": 2.0,
               "receive_cooldown_s": 6.0, "bend_rearm_frames": 6,
               "owner_lost_abort_s": 2.0},
    "classes": {"owner_conf_min": 0.5,
                "hold_classes": ["bottle", "cup", "wine glass", "banana",
                                 "apple", "orange", "handbag", "backpack",
                                 "teddy bear", "suitcase"],
                "hold_iou_min": 0.05, "hold_conf_min": 0.30,
                "held_latch_s": 1.5, "held_min_hits": 1,
                "hold_any_class": False, "hold_any_min_conf": 0.5,
                "hold_any_top_frac": 0.45, "hold_any_max_area_ratio": 0.3,
                "hold_named_max_area_ratio": 0.6},
    "paths": {"grid_json": "~/world_lab/fusion/grid.json"},
    "pose": {"enabled": False, "json": "/dev/shm/pose_out.json",
             "max_age_s": 1.5, "iou_min": 0.1},
}


def load_params(path=None):
    """合并默认值 + 参数文件(一层深合并); 文件缺失也能跑"""
    import copy
    p = copy.deepcopy(DEFAULTS)
    path = path or os.environ.get("HRI_PARAMS") or DEFAULT_PARAMS
    try:
        with open(path) as f:
            user = json.load(f)
        for section, val in user.items():
            if isinstance(val, dict) and isinstance(p.get(section), dict):
                p[section].update(val)
            else:
                p[section] = val
        p["_source"] = path
    except (OSError, ValueError):
        p["_source"] = "builtin-defaults"
    return p


def iou_ratio(inner, outer):
    ix1, iy1 = max(inner[0], outer[0]), max(inner[1], outer[1])
    ix2, iy2 = min(inner[2], outer[2]), min(inner[3], outer[3])
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    a_in = max(1e-6, (inner[2] - inner[0]) * (inner[3] - inner[1]))
    return (iw * ih) / a_in


def pick_safe_spot(grid_path):
    """消费 [世界] grid.json v1: 门优先, 其次最靠正前的自由方向"""
    try:
        with open(os.path.expanduser(grid_path)) as f:
            g = json.load(f)
    except (OSError, ValueError):
        return None, "grid不可读"
    age = time.time() - g.get("ts", 0)
    if age > 300:
        return None, f"grid过期{age:.0f}s"
    # [世界] 交互亲和度打分(优先): 仅 confident 且最优分为正时采用, 否则回退
    sc = g.get("sector_scores") or {}
    if sc.get("confident") and (sc.get("best_score") or 0) > 0 \
            and sc.get("best_bearing_deg") is not None:
        return sc["best_bearing_deg"], f"sector_score:{sc['best_score']:.2f}"
    # 回退: 门优先 -> 自由方向 (兼容 objects[]/free_directions_deg 与 map{})
    doors = [o for o in g.get("objects", [])
             if "door" in o.get("cls", "").lower()]
    if not doors:
        doors = [dict(d, cls="door") for d in (g.get("map", {}) or {}).get("doors", [])]
    if doors:
        d = min(doors, key=lambda o: abs(o.get("bearing_deg", 180)))
        return d["bearing_deg"], f"door@{d.get('dist_m', -1)}m"
    free = g.get("free_directions_deg") or (g.get("map", {}) or {}).get("free_sectors") or []
    if free:
        b = min(free, key=lambda x: abs(x))
        return b, "free"
    return None, "无自由方向"


class HRIStateMachine:
    def __init__(self, send_cmd_fn=None, log_fn=print, params=None):
        self.p = params or load_params()
        self.send_cmd = send_cmd_fn or (lambda cmd: None)
        self.log = log_fn
        cam = self.p["camera"]
        self.Kw = (cam.get("focal_px", 0) * cam.get("shoulder_width_m", 0)) or 0
        self.state = "FOLLOW"
        self.state_since = time.time()
        self.sm_cx = self.sm_cy = None
        self.sm_area = None
        self.sm_h = None
        self.sm_w = None
        self.sm_asp = None
        self.base_asp = None
        self.static_since = None
        self.last_moving_t = 0.0
        self.receive_cooldown_until = 0.0
        self.bend_armed = True
        self.upright_run = 0
        self.hide_bearing = None
        self.owner_last_seen = time.time()
        self.prev_dist_raw = None
        self.last_progress_t = 0.0
        self.held_label = None
        self.held_last_t = 0.0
        self.held_hits = 0
        self.held_armed_until = 0.0
        self.held_conf = 0.0
        self.v_sm = None
        self.recede_since = None
        self.fast_run = 0

    # ---- 内部工具 ----
    def _set_state(self, new, reason=""):
        if new == self.state:
            return
        if new == "HIDE":
            self.hide_bearing, spot_why = pick_safe_spot(
                self.p["paths"]["grid_json"])
            reason += f" | 安全点={self.hide_bearing}({spot_why})"
        self.log(f"[HRI] {self.state} -> {new} ({reason})")
        self.state = new
        self.state_since = time.time()
        self.static_since = None

    def _dwell_ok(self):
        return time.time() - self.state_since >= self.p["timing"]["dwell_s"]

    def _pick_owner(self, dets, score_fn):
        cands = []
        floor = self.p["classes"]["owner_conf_min"]
        for d in dets:
            if d.get("label") != "person":
                continue
            s = score_fn(d) if score_fn else d.get("conf", 0)
            if s > floor:
                cands.append((s, d))
        if not cands:
            return None
        best_s = max(s for s, _ in cands)
        # 连续性优先: 分数接近的候选里, 选离上一帧主人中心最近的, 避免双人跳变
        near = [(s, d) for s, d in cands if s >= best_s - 0.15]
        last_cx = getattr(self, "last_owner_cx", None)
        if last_cx is not None and len(near) > 1:
            best = min(near, key=lambda t: abs((t[1]["bbox"][0] + t[1]["bbox"][2]) / 2 - last_cx))[1]
        else:
            best = max(near, key=lambda t: t[0])[1]
        self.last_owner_cx = (best["bbox"][0] + best["bbox"][2]) / 2
        return best

    def _pose_offering(self, owner_box, now):
        """[世界] pose_server v2 的 offering 作为 OR 辅助通道(默认关)."""
        cfg = self.p.get("pose", {})
        if not cfg.get("enabled"):
            return False
        try:
            with open(os.path.expanduser(cfg["json"])) as f:
                po = json.load(f)
        except (OSError, ValueError):
            return False
        if now - (po.get("ts", 0)) > cfg.get("max_age_s", 1.5):
            return False
        for p in po.get("persons", []):
            if not p.get("offering") or p.get("facing") != "front":
                continue
            if iou_ratio(p.get("bbox", [0, 0, 0, 0]), owner_box) >= cfg.get("iou_min", 0.1):
                return True
        return False

    def _held_object(self, dets, owner_box):
        """具名类别优先, 再走类别无关关系判据; 两者都要求:
        与主人上半区重叠 + 面积明显小于主人(排除椅子/盆栽等场景物)"""
        cls = self.p["classes"]
        x1, y1, x2, y2 = owner_box
        owner_area = max(1.0, (x2 - x1) * (y2 - y1))
        upper = [x1, y1, x2, y1 + (y2 - y1) * cls["hold_any_top_frac"]]
        max_area_named = owner_area * cls.get("hold_named_max_area_ratio", 0.6)
        max_area_any = owner_area * cls["hold_any_max_area_ratio"]

        def in_region(d, region, max_area):
            if "bbox" not in d:
                return False
            bx1, by1, bx2, by2 = d["bbox"]
            if max(1.0, (bx2 - bx1) * (by2 - by1)) > max_area:
                return False
            return iou_ratio(d["bbox"], region) >= cls["hold_iou_min"]

        for d in dets:
            # 具名类: 整个人框范围(支持"手提袋挂在身侧"这种真实姿态), 面积<=0.6x主人
            if (d.get("label") in cls["hold_classes"]
                    and d.get("conf", 0) >= cls["hold_conf_min"]
                    and in_region(d, owner_box, max_area_named)):
                return d["label"], d.get("conf", 0)
        if cls.get("hold_any_class"):
            for d in dets:
                if d.get("label") == "person":
                    continue
                if d.get("conf", 0) < cls["hold_any_min_conf"]:
                    continue
                if in_region(d, upper, max_area_any):
                    return f"obj:{d['label']}", d.get("conf", 0)
        return None, 0.0

    # ---- 主入口: 每帧喂一次 ----
    def feed(self, dets, owner_score_fn=None, robot_moving=False):
        p = self.p
        now = time.time()
        if robot_moving:
            self.last_moving_t = now
        evidence_ok = (now - self.last_moving_t) >= p["timing"]["ego_quiet_s"]

        owner = self._pick_owner(dets, owner_score_fn)
        act = "NONE"
        if owner is None:
            lost = time.time() - self.owner_last_seen
            if self.state == "HIDE" and lost > p["timing"]["owner_lost_abort_s"]:
                self._set_state("FOLLOW", "主人消失, 扫掠中断")
            elif (self.state in ("RECEIVE", "YIELD")
                  and lost > p["timing"]["owner_lost_abort_s"]):
                self.receive_cooldown_until = now + p["timing"]["receive_cooldown_s"]
                self._set_state("WAIT", "主人离开")
            return {"state": self.state, "action": "NONE"}
        self.owner_last_seen = time.time()

        x1, y1, x2, y2 = owner["bbox"]
        cx, cy, area = (x1 + x2) / 2, (y1 + y2) / 2, max(1.0, (x2 - x1) * (y2 - y1))
        h = max(1.0, y2 - y1)
        w = max(1.0, x2 - x1)
        clip_h = p["camera"]["frame_h"] * p["camera"]["clip_ratio"]
        clipped = h >= clip_h

        disp = rate = dcx = 0.0
        dist_w = self.Kw / w if self.Kw else None
        # 接近速率不受自运动门控: RECEIVE 会亲自驱动车前进,
        # 若被门控屏蔽, 动作本身会把自己判成"停滞"而提前放弃。
        approach_mps = None
        if dist_w is not None and self.prev_dist_raw is not None:
            dt = max(1e-3, now - getattr(self, "_last_t", now))
            approach_mps = (self.prev_dist_raw - dist_w) / dt
            vmax = p["metric"].get("v_max_mps", 2.5)   # 夹掉双人跳变造成的爆表速度
            approach_mps = max(-vmax, min(vmax, approach_mps))
            ema = p["metric"]["v_ema"]
            self.v_sm = (ema * approach_mps + (1 - ema) * self.v_sm
                         if self.v_sm is not None else approach_mps)
        held, held_conf = None, 0.0
        bend = False
        if evidence_ok and self.sm_area is not None:
            dt = max(1e-3, now - getattr(self, "_last_t", now))
            disp = ((cx - self.sm_cx) ** 2 + (cy - self.sm_cy) ** 2) ** 0.5
            dcx = abs(cx - getattr(self, "_raw_cx", cx))
            rate = (area / self.sm_area - 1.0) / dt
            held, held_conf = self._held_object(dets, owner["bbox"])
            if self.base_asp and not clipped:
                bend = self.sm_asp < self.base_asp * (1 - p["ratios"]["bend"])
                depressed = self.sm_asp < self.base_asp * (1 - p["ratios"]["bend_clear"])
            else:
                depressed = False
            if depressed:
                self.upright_run = 0
            else:
                self.upright_run += 1
                if self.upright_run >= p["timing"]["bend_rearm_frames"]:
                    self.bend_armed = True
        # 手持物: 需连续命中 held_min_hits 帧才武装, 之后 latch 窗口内保持
        if held:
            self.held_last_t = now
            self.held_hits += 1
            if self.held_hits >= p["classes"]["held_min_hits"]:
                self.held_label, self.held_conf = held, held_conf
                self.held_armed_until = now + p["classes"]["held_latch_s"]
        else:
            self.held_hits = 0
        if now < self.held_armed_until:
            held, held_conf = self.held_label, self.held_conf
        else:
            held, held_conf = None, 0.0
        self._raw_cx = cx
        self.prev_dist_raw = dist_w

        a = 0.35
        self.sm_cx = a * cx + (1 - a) * (self.sm_cx if self.sm_cx is not None else cx)
        self.sm_cy = a * cy + (1 - a) * (self.sm_cy if self.sm_cy is not None else cy)
        self.sm_area = a * area + (1 - a) * (self.sm_area or area)
        self.sm_h = 0.5 * h + 0.5 * (self.sm_h or h)
        w = max(1.0, x2 - x1)
        self.sm_w = 0.5 * w + 0.5 * (self.sm_w or w)
        if not clipped:  # 裁切时姿态信号无效, 冻结避免污染基线
            asp = h / w
            self.sm_asp = 0.5 * asp + 0.5 * (self.sm_asp or asp)
            if self.base_asp is None:
                self.base_asp = self.sm_asp
            elif self.sm_asp > self.base_asp:
                self.base_asp += p["ratios"]["base_up_alpha"] * (self.sm_asp - self.base_asp)
            else:
                self.base_asp += p["ratios"]["base_down_alpha"] * (self.sm_asp - self.base_asp)
        self._last_t = now

        # ---- 距离/速度全部用宽度反推(姿态免疫); 裁切单独判"很近" ----
        dist_sm = self.Kw / self.sm_w if (self.Kw and self.sm_w) else None
        dist_ok = dist_sm is not None
        speed_ok = dist_ok and approach_mps is not None
        if dist_ok:
            dcx_m = dcx * dist_sm / p["camera"]["focal_px"]
            v_now = approach_mps if approach_mps is not None else 0.0
            static_now = (dcx_m < p["metric"]["static_disp_m"]
                          and abs(v_now) < p["metric"]["static_v_mps"])
            wake_now = dcx_m > p["metric"]["wake_disp_m"]
        else:
            static_now = disp < p["pixel"]["static_disp_px"]
            wake_now = dcx > p["pixel"]["wake_disp_px"]
        # 弯腰距离闸单独放宽(需全身可见才判姿态, 裁切时不判, 故不用 clipped 兜底)
        near_bend = dist_ok and dist_sm <= p["metric"]["bend_near_dist_m"]
        v_sm = self.v_sm if self.v_sm is not None else (approach_mps or 0.0)
        if speed_ok:
            thr = p["metric"]["approach_fast_mps"]
            self.fast_run = self.fast_run + 1 if approach_mps >= thr else 0
            # 需连续>=2帧(约0.6s)且平滑速度也够, 单帧宽度噪声过不了
            fast = self.fast_run >= 2 and v_sm >= thr * 0.6
            slow = (p["metric"]["approach_slow_mps"] <= v_sm
                    < p["metric"]["approach_fast_mps"])
            receding = v_sm < -p["metric"]["recede_mps"]
        else:
            fast = rate > p["pixel"]["approach_fast_rate"]
            slow = p["pixel"]["approach_slow_rate"] < rate <= p["pixel"]["approach_fast_rate"]
            receding = rate < -0.05

        if evidence_ok and static_now:
            self.static_since = self.static_since or now
        elif not static_now:
            self.static_since = None

        # RECEIVE 超时以"最近一次有意义的进展"为基准(>recede 阈), 避免抖动刷计时
        progressing = fast or v_sm > p["metric"]["recede_mps"]
        if progressing:
            self.last_progress_t = now

        s = self.state
        approaching_fast = evidence_ok and fast
        bend_go = (bend and near_bend and self.bend_armed
                   and now >= self.receive_cooldown_until)
        held_go = (held and slow and now >= self.receive_cooldown_until)
        pose_go = (evidence_ok and now >= self.receive_cooldown_until
                   and self._pose_offering(owner["bbox"], now))
        offering = evidence_ok and (held_go or bend_go or pose_go)
        if s == "FOLLOW":
            if evidence_ok and static_now:
                if now - self.static_since >= p["timing"]["static_need_s"] and self._dwell_ok():
                    self._set_state("WAIT", f"主人静止{p['timing']['static_need_s']}s")
        elif s in ("WAIT", "HIDE"):
            if approaching_fast and self._dwell_ok():
                self._set_state("YIELD", f"快速靠近 {self._speed_str(approach_mps, rate)}")
            elif offering and self._dwell_ok():
                if bend_go and not held:
                    self.bend_armed = False
                self.receive_cooldown_until = now + p["timing"]["receive_cooldown_s"]
                if pose_go and not held and not bend_go:
                    why = "pose offering"
                elif bend_go and not held:
                    why = "弯腰姿态"
                else:
                    why = f"持物靠近 {held}:{held_conf:.2f}"
                self.last_progress_t = now
                self._set_state("RECEIVE", why)
            elif wake_now and not held and not bend and self._dwell_ok():
                self._set_state("FOLLOW", "主人恢复移动")
            elif (s == "WAIT" and self.static_since
                  and now - self.static_since >= p["timing"]["hide_after_s"]
                  and self._dwell_ok()):
                self._set_state("HIDE", f"主人持续静止{p['timing']['hide_after_s']:.0f}s")
            elif s == "HIDE":
                act = (f"GOTO_SAFE {self.hide_bearing:.0f}"
                       if self.hide_bearing is not None else "GOTO_SAFE")
        elif s == "YIELD":
            if not fast and self._dwell_ok():
                self._set_state("WAIT", "靠近结束")
            else:
                act = "BACK_OFF"
        elif s == "RECEIVE":
            if owner is None:
                self.receive_cooldown_until = now + p["timing"]["receive_cooldown_s"]
                self._set_state("WAIT", "主人离开")
            elif now - self.state_since < p["timing"]["dwell_s"]:
                act = "APPROACH"  # 最小驻留, 不被首帧噪声踢出
            elif receding:
                self.recede_since = self.recede_since or now
                if now - self.recede_since >= p["metric"]["receive_exit_s"]:
                    self.receive_cooldown_until = now + p["timing"]["receive_cooldown_s"]
                    self._set_state("WAIT", "交接完成(持续后退)")
                else:
                    act = "APPROACH"
            else:
                self.recede_since = None
                if now - self.last_progress_t > p["timing"]["receive_timeout_s"]:
                    self.receive_cooldown_until = now + p["timing"]["receive_cooldown_s"]
                    self._set_state("WAIT", "逼近停滞/已到位")
                else:
                    act = "APPROACH"
        self.send_cmd(act)
        return {"state": self.state, "action": act, "disp": round(disp, 1),
                "dcx": round(dcx, 1), "rate": round(rate, 3),
                "dist_m": round(dist_sm, 2) if dist_sm else None,
                "v_mps": round(v_sm, 2),
                "v_raw": round(approach_mps, 2) if approach_mps is not None else None,
                "held": held, "bend": bend,
                "mode": "clip" if clipped else ("metric" if dist_ok else "pixel")}

    @staticmethod
    def _speed_str(v, rate):
        return f"v={v:.2f}m/s" if v is not None else f"rate={rate:.2f}"


def load_dets_api(url="http://127.0.0.1:8080/api/detections"):
    with urllib.request.urlopen(url, timeout=1) as r:
        raw = json.load(r).get("detections", [])
    out = []
    for d in raw:
        try:
            out.append({"label": d["c"], "conf": float(d.get("p", 0)),
                        "bbox": [d["x1"], d["y1"], d["x2"], d["y2"]]})
        except (KeyError, ValueError, TypeError):
            continue
    return out


def run_live(params_path=None):
    params = load_params(params_path)
    logdir = os.path.expanduser("~/hri_logs")
    os.makedirs(logdir, exist_ok=True)
    logpath = os.path.join(logdir, time.strftime("hri_%Y%m%d_%H%M%S.jsonl"))
    logfile = open(logpath, "a", buffering=1)
    t0 = time.time()
    print(f"[HRI] 黑匣子: {logpath} | 参数: {params.get('_source')}")
    logfile.write(json.dumps({"t0": t0, "type": "session_start",
                              "params": params.get("_source")}) + "\n")

    def log_and_record(msg):
        logfile.write(json.dumps(
            {"t": round(time.time() - t0, 2), "type": "transition",
             "msg": msg}, ensure_ascii=False) + "\n")
        print(msg)

    sm = HRIStateMachine(log_fn=log_and_record, params=params)
    while True:
        try:
            dets = load_dets_api()
            out = sm.feed(dets)
            out = {**out, "dets": [[d["label"], round(d["conf"], 2)] for d in dets]}
            logfile.write(json.dumps(
                {"t": round(time.time() - t0, 2), "type": "frame", **out},
                ensure_ascii=False) + "\n")
            print(json.dumps(out, ensure_ascii=False))
        except Exception as e:
            logfile.write(json.dumps(
                {"t": round(time.time() - t0, 2),
                 "type": "error", "err": str(e)}) + "\n")
            print(f"[HRI] api err: {e}")
        time.sleep(0.3)


def run_fake(path, params_path=None):
    sm = HRIStateMachine(params=load_params(params_path))
    with open(path) as f:
        for line in f:
            frame = json.loads(line)
            time.sleep(frame.get("wait", 0))
            out = sm.feed(frame["dets"], robot_moving=frame.get("moving", False))
            print(json.dumps(out, ensure_ascii=False))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--fake", help="假数据脚本 jsonl: {wait,dets,moving}")
    ap.add_argument("--params", help="参数文件路径 (默认 hri_params.json)")
    args = ap.parse_args()
    if args.live:
        run_live(args.params)
    else:
        run_fake(args.fake, args.params)
