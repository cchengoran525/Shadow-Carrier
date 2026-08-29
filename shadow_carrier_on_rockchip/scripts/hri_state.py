#!/usr/bin/env python3
"""hri_state.py v0.1 - [HRI] 行为状态机骨架
状态: FOLLOW / WAIT / HIDE / RECEIVE / YIELD
数据源: /api/detections + owner_score() [认主线冻结接口]
门控: 自运动期间(车在动)不采信 bbox 位移证据
用法:
  真数据: python3 hri_state.py --live
  假数据: python3 hri_state.py --fake fake_script.jsonl
"""
import time, json, argparse, os, urllib.request

# ========== 调参区 ==========
OWNER_CONF_MIN = 0.5      # 与认主线 CONF_MIN 对齐
STATIC_DISP_PX = 15       # 主人"静止"判定: bbox中心位移阈值
STATIC_NEED_S = 5.0       # 持续静止多久 -> WAIT
WAKE_DISP_PX = 40         # WAIT中主人位移超过此值 -> 恢复FOLLOW
AREA_FAST_RATE = 1.2      # bbox面积每秒增长倍率 > 此值 = 快速靠近 -> YIELD
AREA_SLOW_RATE = 0.08     # 缓慢靠近下限, 区间内+持物 = RECEIVE
HOLD_CLASSES = {"bottle", "cup", "wine glass", "banana", "apple", "orange",
                "handbag", "backpack"}
HOLD_IOU_MIN = 0.05       # 手持物与主人bbox的重叠占比门槛
HOLD_CONF_MIN = 0.35      # 手持物最低置信度(过滤全画面噪声类)
BEND_RATIO = 0.25         # bbox高度比慢基线低25%以上 = 弯腰姿态
BASE_UP_ALPHA = 0.15      # 基线快速上抬(站起/走近立即刷新)
BASE_DOWN_ALPHA = 0.005   # 基线极慢下降(蹲下弯腰不拖低基线)
NEAR_H_MIN = 300          # 弯腰触发距离闸: bbox高≥300px(约1m内)
RECEIVE_TIMEOUT_S = 2.0   # RECEIVE后无进一步逼近则放弃
RECEIVE_COOLDOWN_S = 6.0  # RECEIVE结束后冷却期内弯腰不再触发(防蹲看振荡)
BEND_REARM_FRAMES = 6     # 弯腰结束后需连续N帧直立才重新武装
BEND_CLEAR_RATIO = 0.10   # 压低超过10%即视为仍在弯腰序列(施密特下阈)
DWELL_S = 1.0             # 状态最小驻留, 防抖动
EGO_QUIET_S = 1.0         # 车停止后需安静这么久才开始采信bbox
# ============================

STATES = ("FOLLOW", "WAIT", "HIDE", "RECEIVE", "YIELD")


def iou_ratio(inner, outer):
    ix1, iy1 = max(inner[0], outer[0]), max(inner[1], outer[1])
    ix2, iy2 = min(inner[2], outer[2]), min(inner[3], outer[3])
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    a_in = max(1e-6, (inner[2] - inner[0]) * (inner[3] - inner[1]))
    return (iw * ih) / a_in


class HRIStateMachine:
    def __init__(self, send_cmd_fn=None, log_fn=print):
        self.send_cmd = send_cmd_fn or (lambda cmd: None)
        self.log = log_fn
        self.state = "FOLLOW"
        self.state_since = time.time()
        self.sm_cx = self.sm_cy = None
        self.sm_area = None
        self.sm_h = None
        self.base_h = None
        self.static_since = None
        self.last_moving_t = 0.0
        self.receive_cooldown_until = 0.0
        self.bend_armed = True
        self.upright_run = 0

    # ---- 内部工具 ----
    def _set_state(self, new, reason=""):
        if new == self.state:
            return
        self.log(f"[HRI] {self.state} -> {new} ({reason})")
        self.state = new
        self.state_since = time.time()
        self.static_since = None

    def _dwell_ok(self):
        return time.time() - self.state_since >= DWELL_S

    def _pick_owner(self, dets, score_fn):
        best, best_s = None, OWNER_CONF_MIN
        for d in dets:
            if d.get("label") != "person":
                continue
            s = score_fn(d) if score_fn else d.get("conf", 0)
            if s > best_s:
                best, best_s = d, s
        return best

    def _held_object(self, dets, owner_box):
        for d in dets:
            if (d.get("label") in HOLD_CLASSES and "bbox" in d
                    and d.get("conf", 0) >= HOLD_CONF_MIN):
                if iou_ratio(d["bbox"], owner_box) >= HOLD_IOU_MIN:
                    return d["label"], d.get("conf", 0)
        return None, 0.0

    # ---- 主入口: 每帧喂一次 ----
    def feed(self, dets, owner_score_fn=None, robot_moving=False):
        now = time.time()
        if robot_moving:
            self.last_moving_t = now
        evidence_ok = (now - self.last_moving_t) >= EGO_QUIET_S

        owner = self._pick_owner(dets, owner_score_fn)
        act = "NONE"
        if owner is None:
            return {"state": self.state, "action": "NONE"}

        x1, y1, x2, y2 = owner["bbox"]
        cx, cy, area = (x1 + x2) / 2, (y1 + y2) / 2, max(1.0, (x2 - x1) * (y2 - y1))

        disp = rate = dcx = 0.0
        held, held_conf = None, 0.0
        bend = False
        if evidence_ok and self.sm_area is not None:
            dt = max(1e-3, now - getattr(self, "_last_t", now))
            disp = ((cx - self.sm_cx) ** 2 + (cy - self.sm_cy) ** 2) ** 0.5
            dcx = abs(cx - getattr(self, "_raw_cx", cx))
            rate = (area / self.sm_area - 1.0) / dt
            held, held_conf = self._held_object(dets, owner["bbox"])
            if self.base_h:
                bend = self.sm_h < self.base_h * (1 - BEND_RATIO)
                depressed = self.sm_h < self.base_h * (1 - BEND_CLEAR_RATIO)
            else:
                depressed = False
            if depressed:
                self.upright_run = 0
            else:
                self.upright_run += 1
                if self.upright_run >= BEND_REARM_FRAMES:
                    self.bend_armed = True
        self._raw_cx = cx

        a = 0.35
        self.sm_cx = a * cx + (1 - a) * (self.sm_cx if self.sm_cx is not None else cx)
        self.sm_cy = a * cy + (1 - a) * (self.sm_cy if self.sm_cy is not None else cy)
        self.sm_area = a * area + (1 - a) * (self.sm_area or area)
        h = max(1.0, y2 - y1)
        self.sm_h = 0.5 * h + 0.5 * (self.sm_h or h)
        if self.base_h is None:
            self.base_h = self.sm_h
        elif self.sm_h > self.base_h:
            self.base_h += BASE_UP_ALPHA * (self.sm_h - self.base_h)
        else:
            self.base_h += BASE_DOWN_ALPHA * (self.sm_h - self.base_h)
        self._last_t = now

        s = self.state
        approaching_fast = evidence_ok and rate > AREA_FAST_RATE
        near = self.sm_h is not None and self.sm_h >= NEAR_H_MIN
        bend_go = (bend and near and self.bend_armed
                   and now >= self.receive_cooldown_until)
        offering = evidence_ok and (
            (held and AREA_SLOW_RATE < rate <= AREA_FAST_RATE) or bend_go)
        if s == "FOLLOW":
            if evidence_ok and disp < STATIC_DISP_PX:
                self.static_since = self.static_since or now
                if now - self.static_since >= STATIC_NEED_S and self._dwell_ok():
                    self._set_state("WAIT", f"主人静止{STATIC_NEED_S}s")
            elif disp >= STATIC_DISP_PX:
                self.static_since = None
        elif s in ("WAIT", "HIDE"):
            if approaching_fast and self._dwell_ok():
                self._set_state("YIELD", f"快速靠近 rate={rate:.2f}")
            elif offering and self._dwell_ok():
                if bend_go and not held:
                    self.receive_cooldown_until = now + RECEIVE_COOLDOWN_S
                    self.bend_armed = False
                why = "弯腰姿态" if (bend_go and not held) else f"持物靠近 {held}:{held_conf:.2f}"
                self._set_state("RECEIVE", why)
            elif dcx > WAKE_DISP_PX and not held and not bend and self._dwell_ok():
                self._set_state("FOLLOW" if s == "WAIT" else s, "主人恢复移动")
        elif s == "YIELD":
            if rate <= AREA_FAST_RATE and self._dwell_ok():
                self._set_state("WAIT", "靠近结束")
            else:
                act = "BACK_OFF"
        elif s == "RECEIVE":
            if owner is None:
                self._set_state("WAIT", "物品交接完成/主人离开")
            elif rate < -0.05:
                self._set_state("WAIT", "交接完成")
            elif time.time() - self.state_since > RECEIVE_TIMEOUT_S and rate <= 0:
                self._set_state("WAIT", "逼近停滞, 疑似误触发")
            else:
                act = "APPROACH"
        elif s == "HIDE":
            if disp > WAKE_DISP_PX:
                self._set_state("FOLLOW", "主人来找")
        self.send_cmd(act)
        return {"state": self.state, "action": act, "disp": round(disp, 1),
                "rate": round(rate, 3), "held": held, "bend": bend}


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


def run_live():
    logdir = os.path.expanduser("~/hri_logs")
    os.makedirs(logdir, exist_ok=True)
    logpath = os.path.join(logdir, time.strftime("hri_%Y%m%d_%H%M%S.jsonl"))
    logfile = open(logpath, "a", buffering=1)
    t0 = time.time()
    print(f"[HRI] 黑匣子: {logpath}")
    logfile.write(json.dumps({"t0": t0, "type": "session_start"}) + "\n")

    def log_and_record(msg):
        logfile.write(json.dumps(
            {"t": round(time.time() - t0, 2), "type": "transition",
             "msg": msg}, ensure_ascii=False) + "\n")
        print(msg)

    sm = HRIStateMachine(log_fn=log_and_record)
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


def run_fake(path):
    sm = HRIStateMachine()
    with open(path) as f:
        for line in f:
            frame = json.loads(line)
            t0 = frame.get("wait", 0)
            time.sleep(t0)
            out = sm.feed(frame["dets"], robot_moving=frame.get("moving", False))
            print(json.dumps(out, ensure_ascii=False))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--fake", help="假数据脚本 jsonl: {wait,dets,moving}")
    args = ap.parse_args()
    run_live() if args.live else run_fake(args.fake)
