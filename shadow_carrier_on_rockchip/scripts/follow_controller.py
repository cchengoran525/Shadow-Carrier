#!/usr/bin/env python3
"""follow_controller.py v3 - 差速弧线跟人: DIFF L<左速> R<右速>"""
import time, json, urllib.request

# ========== 调参区 ==========
FW, FH = 640, 480
FCX = FW / 2

ALPHA = 0.3           # EMA 平滑

DIR_DEADBAND = 50     # 死区: ±50px内直走

BASE_SPD = 100        # 基准前进速度
MIN_SPD = 60          # 最小轮速 (一侧降到这个速度时已是急转)
MAX_OFFSET = 200       # 偏移饱和点

TARGET_BBOX_H = 400
DIST_DEADBAND = 40
MIN_BBOX_H = 150
FWD_SPD = 100

H_CHANGE_LIMIT = 0.3

CYCLE_S = 0.3         # 控制周期 300ms
LOST_LIMIT = 6
BACK_SPD = 85
BACK_MS = 0.25

DET_API = "http://127.0.0.1:8080/api/detections"
# ============================

class FollowController:
    def __init__(self, send_cmd_fn):
        self.send_cmd = send_cmd_fn
        self.scx = FCX
        self.scy = FH / 2
        self.sh = 0
        self.lost = 0
        self.paused = False
        self.running = False
        self.last_turn_dir = None
        self.backing = False

    def start(self):
        self.running = True; self.lost = 0
        self.last_turn_dir = None; self.backing = False
        print("[follow] started")

    def stop(self):
        self.running = False; self.send_cmd("STOP")
        print("[follow] stopped")

    def pause(self):
        self.paused = True; self.send_cmd("STOP")
        print("[follow] paused")

    def resume(self):
        self.paused = False; self.lost = 0
        self.last_turn_dir = None
        print("[follow] resumed")

    def _fetch_person(self):
        try:
            req = urllib.request.Request(DET_API, headers={"User-Agent": "follow"})
            d = json.loads(urllib.request.urlopen(req, timeout=1.0).read())
        except Exception:
            return None
        best = None
        for det in d.get("detections", []):
            if det.get("c") != "person": continue
            area = (det["x2"] - det["x1"]) * (det["y2"] - det["y1"])
            if best is None or area > best["area"]:
                best = {"cx": (det["x1"] + det["x2"]) / 2,
                        "h": det["y2"] - det["y1"],
                        "cy": (det["y1"] + det["y2"]) / 2, "area": area}
        return best

    def _diff(self, left, right):
        """发送差速命令"""
        self.send_cmd(f"DIFF L{left} R{right}")

    def tick(self):
        if not self.running: return None, None, {"state": "stopped"}
        if self.paused: return None, None, {"state": "paused"}

        # 回退中
        if self.backing:
            self.send_cmd("STOP")
            self.backing = False
            self.last_turn_dir = None
            return "STOP", 0, {"state": "back_done"}

        person = self._fetch_person()

        if person is None:
            self.lost += 1
            if self.lost >= LOST_LIMIT:
                self.send_cmd("STOP")
                self.last_turn_dir = None
                return "STOP", 0, {"state": "lost_stop"}
            if self.last_turn_dir and self.lost == 1:
                self.send_cmd(f"MOVE B {BACK_SPD}")
                self.backing = True
                self.last_turn_dir = None
                return "B", BACK_SPD, {"state": "backing"}
            return None, None, {"state": "lost", "count": self.lost}

        self.lost = 0
        raw_cx, raw_h = person["cx"], person["h"]

        # EMA 平滑
        if self.sh == 0:
            self.scx, self.scy, self.sh = raw_cx, person["cy"], raw_h
        else:
            h_change = abs(raw_h - self.sh) / self.sh if self.sh > 0 else 0
            if h_change < H_CHANGE_LIMIT:
                self.scx = ALPHA * raw_cx + (1 - ALPHA) * self.scx
                self.scy = ALPHA * person["cy"] + (1 - ALPHA) * self.scy
                self.sh  = ALPHA * raw_h + (1 - ALPHA) * self.sh

        offset = self.scx - FCX
        abs_off = abs(offset)

        # === 差速映射: 偏移→左右轮速 ===
        if abs_off <= DIR_DEADBAND:
            # 直走
            left = right = BASE_SPD
            self.last_turn_dir = None
        else:
            # 弧线: 一侧降速
            ratio = min(abs_off / MAX_OFFSET, 1.0)
            slow = int(BASE_SPD - ratio * (BASE_SPD - MIN_SPD))
            if offset < 0:
                # 人在左→左轮慢, 右轮快=左弧
                left, right = slow, BASE_SPD
            else:
                # 人在右→右轮慢, 左轮快=右弧
                left, right = BASE_SPD, slow
            self.last_turn_dir = "L" if offset < 0 else "R"

        self._diff(left, right)
        return "DIFF", f"L{left}R{right}", self._info(offset)

    def _info(self, offset):
        return {"state": "active", "paused": self.paused,
                "smooth_cx": round(self.scx, 1), "smooth_h": round(self.sh, 1),
                "offset": round(offset, 1), "lost": self.lost}


def run_follow_loop(controller):
    while controller.running:
        controller.tick()
        time.sleep(CYCLE_S)
