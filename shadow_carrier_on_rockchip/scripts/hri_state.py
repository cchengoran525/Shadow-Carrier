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
               "frame_h": 480, "clip_ratio": 0.9, "shoulder_width_m": 0.45},
    "metric": {"static_disp_m": 0.08, "wake_disp_m": 0.40, "near_dist_m": 1.2,
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
                                 "apple", "orange", "handbag", "backpack"],
                "hold_iou_min": 0.05, "hold_conf_min": 0.35,
                "hold_any_class": True, "hold_any_min_conf": 0.4,
                "hold_any_top_frac": 0.6, "hold_any_max_area_ratio": 0.6},
    "paths": {"grid_json": "~/world_lab/fusion/grid.json"},
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
    doors = [o for o in g.get("objects", [])
             if "door" in o.get("cls", "").lower()]
    if doors:
        d = min(doors, key=lambda o: abs(o.get("bearing_deg", 180)))
        return d["bearing_deg"], f"door:{d['cls']}@{d['dist_m']:.1f}m"
    free = g.get("free_directions_deg") or []
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
        best, best_s = None, self.p["classes"]["owner_conf_min"]
        for d in dets:
            if d.get("label") != "person":
                continue
            s = score_fn(d) if score_fn else d.get("conf", 0)
            if s > best_s:
                best, best_s = d, s
        return best

    def _held_object(self, dets, owner_box):
        """先按具名类别, 再走类别无关的"手中物"关系判据"""
        cls = self.p["classes"]
        for d in dets:
            if (d.get("label") in cls["hold_classes"] and "bbox" in d
                    and d.get("conf", 0) >= cls["hold_conf_min"]):
                if iou_ratio(d["bbox"], owner_box) >= cls["hold_iou_min"]:
                    return d["label"], d.get("conf", 0)
        if cls.get("hold_any_class"):
            x1, y1, x2, y2 = owner_box
            upper = [x1, y1, x2, y1 + (y2 - y1) * cls["hold_any_top_frac"]]
            owner_area = max(1.0, (x2 - x1) * (y2 - y1))
            for d in dets:
                if d.get("label") == "person" or "bbox" not in d:
                    continue
                if d.get("conf", 0) < cls["hold_any_min_conf"]:
                    continue
                bx1, by1, bx2, by2 = d["bbox"]
                obj_area = max(1.0, (bx2 - bx1) * (by2 - by1))
                if obj_area > owner_area * cls.get("hold_any_max_area_ratio", 0.6):
                    continue  # 全画面误检框(如"train"), 不是手中物
                if iou_ratio(d["bbox"], upper) >= cls["hold_iou_min"]:
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
            if (self.state == "HIDE" and
                    time.time() - self.owner_last_seen > p["timing"]["owner_lost_abort_s"]):
                self._set_state("FOLLOW", "主人消失, 扫掠中断")
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
            static_now = dcx_m < p["metric"]["static_disp_m"]
            wake_now = dcx_m > p["metric"]["wake_disp_m"]
        else:
            static_now = disp < p["pixel"]["static_disp_px"]
            wake_now = dcx > p["pixel"]["wake_disp_px"]
        # 距离闸用宽度反推(姿态免疫): 弯腰只改高度不改肩宽
        near = (dist_ok and dist_sm <= p["metric"]["near_dist_m"]) or clipped
        if speed_ok:
            fast = approach_mps >= p["metric"]["approach_fast_mps"]
            slow = (p["metric"]["approach_slow_mps"] <= approach_mps
                    < p["metric"]["approach_fast_mps"])
            receding = approach_mps < -p["metric"]["recede_mps"]
        else:
            fast = rate > p["pixel"]["approach_fast_rate"]
            slow = p["pixel"]["approach_slow_rate"] < rate <= p["pixel"]["approach_fast_rate"]
            receding = rate < -0.05

        if evidence_ok and static_now:
            self.static_since = self.static_since or now
        elif not static_now:
            self.static_since = None

        # RECEIVE 超时以"最近一次有进展"为基准, 避免接近中被误判停滞
        progressing = (approach_mps is not None and approach_mps > 0) or fast
        if progressing:
            self.last_progress_t = now

        s = self.state
        approaching_fast = evidence_ok and fast
        bend_go = (bend and near and self.bend_armed
                   and now >= self.receive_cooldown_until)
        held_go = (held and slow and now >= self.receive_cooldown_until)
        offering = evidence_ok and (held_go or bend_go)
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
                why = "弯腰姿态" if (bend_go and not held) else f"持物靠近 {held}:{held_conf:.2f}"
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
            if owner is None or receding:
                self.receive_cooldown_until = now + p["timing"]["receive_cooldown_s"]
                self._set_state("WAIT", "交接完成/主人离开")
            elif now - self.last_progress_t > p["timing"]["receive_timeout_s"]:
                self.receive_cooldown_until = now + p["timing"]["receive_cooldown_s"]
                self._set_state("WAIT", "逼近停滞/已到位")
            else:
                act = "APPROACH"
        self.send_cmd(act)
        return {"state": self.state, "action": act, "disp": round(disp, 1),
                "dcx": round(dcx, 1), "rate": round(rate, 3),
                "dist_m": round(dist_sm, 2) if dist_sm else None,
                "v_mps": round(approach_mps, 2) if approach_mps is not None else None,
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
            out = sm.feed(load_dets_api())
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
