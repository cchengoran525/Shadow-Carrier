#!/usr/bin/env python3
"""follow_controller.py v4 - 差速弧线跟人 + 快速认主(颜色+体态, owner_id)"""
import time, json, math, urllib.request
import cv2

# ========== 调参区 ==========
FW, FH = 640, 480
FCX = FW / 2

ALPHA = 0.3           # EMA 平滑

DIR_DEADBAND = 50     # 死区: ±50px内直走

BASE_SPD = 100        # 基准前进速度
MIN_SPD = 60          # 最小轮速 (一侧降到这个速度时已是急转)
MAX_OFFSET = 200       # 偏移饱和点

TARGET_BBOX_H = 400
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
    def __init__(self, send_cmd_fn, bearing_fn=None):
        # bearing_fn: [云台]线提供的"主人世界方位角(度)"只读回调。
        # 非None时转向误差用它(云台解耦), 否则退回原始bbox像素偏差。
        self.bearing_fn = bearing_fn
        self.send_cmd = send_cmd_fn
        self.scx = FCX
        self.scy = FH / 2
        self.sh = 0
        self.lost = 0
        self.paused = False
        self.running = False
        self.last_turn_dir = None
        self.backing = False
        self.profile = None      # 主人模板 (OwnerProfile)
        self.last_score = 0.0
        self.tracker = None      # 单目标α-β跟踪器(认主成功后创建)
        self.low_streak = 0      # 连续低分/丢失拍数(逃逸错锁用)

    def start(self):
        self.running = True; self.lost = 0
        self.last_turn_dir = None; self.backing = False
        # 快速认主: 抓不到模板则降级为"跟最大"
        try:
            import owner_id
            p = owner_id.enroll(log=print)
            self.profile = p
            print(f"[follow] owner lock {'OK' if p else 'FAIL -> largest-fallback'}")
            if p is not None:
                owner_id.start_ble_heartbeat(log=print)  # 手环在场心跳(慢速兜底)
        except Exception as e:
            self.profile = None
            print(f"[follow] owner module error ({e}) -> largest-fallback")
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
        """认主版选人: 有主人模板按 颜色+体态 打分, 否则降级跟最大"""
        try:
            import owner_id
            dets = owner_id._fetch_detections()  # 已做类型防御
        except Exception:
            dets = []
        if not dets:
            return None
        if self.profile is not None:
            try:
                import owner_id
                img = owner_id.read_recent_frame()
                if img is None:
                    return None
                # 长时间丢失后重捕获: 旧预测已失效, 清掉再关联
                if self.lost > LOST_LIMIT and self.tracker is not None:
                    self.tracker.reset()
                box, score = owner_id.select_target(
                    self.profile, img, dets, tracker=self.tracker)
                self.last_score = score
                if box is None:
                    self.low_streak += 1
                    if self.tracker is not None:
                        self.tracker.coast()
                        if self.low_streak >= 8:   # ~3s全低分: 解除错锁偏向
                            self.tracker.reset()
                            self.low_streak = 0
                            print("[follow] re-acquire: tracker reset")
                    # 机主原则(2026-08-28): 模板不匹配 ≠ 没有人。
                    # YOLO person 是唯一先验, 模板只是偏好层 → 降级最大person, 不停车
                    best = None
                    for det in dets:
                        area = (det["x2"] - det["x1"]) * (det["y2"] - det["y1"])
                        if best is None or area > best["area"]:
                            best = {"cx": (det["x1"] + det["x2"]) / 2,
                                    "cy": (det["y1"] + det["y2"]) / 2,
                                    "h": det["y2"] - det["y1"]}
                    if best is not None:
                        return best
                    return None
                cx = (box["x1"] + box["x2"]) / 2
                cy = (box["y1"] + box["y2"]) / 2
                if self.tracker is None:
                    self.tracker = owner_id.TargetTracker()
                self.tracker.update(cx, cy)
                if score < owner_id.SCORE_MIN - 0.05:
                    self.low_streak += 1
                    if self.low_streak >= 8:
                        self.tracker.reset()
                        self.low_streak = 0
                        print("[follow] re-acquire: low-score streak")
                else:
                    self.low_streak = 0
                return {"cx": cx, "h": box["y2"] - box["y1"], "cy": cy}
            except Exception as e:
                print(f"[follow] owner select error: {e}")
                self.profile = None  # 坏了就降级, 别卡死跟随
        best = None
        for det in dets:
            if det.get("c") != "person":     # 只准锁人, 防止大物件(包/箱)抢锁
                continue
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
                # 主人模板在 + 手环或热点任一在场 → 原地等待(不乱跑)
                if self.profile is not None:
                    try:
                        import owner_id
                        if owner_id.owner_nearby():
                            self.send_cmd("STOP")
                            return "STOP", 0, {"state": "wait_owner",
                                               "lost": self.lost}
                    except Exception:
                        pass
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
        # 云台解耦: 世界方位角→等效像素偏差 (FX≈508 来自 state/calib/params.py)
        # 云台边缘保持会把原始bbox误差钳在~108px, 这里把它补回来
        if self.bearing_fn is not None:
            th = self.bearing_fn()
            if th is not None:
                offset = 508.0 * math.tan(math.radians(th))
        abs_off = abs(offset)

        # === 差速映射: 偏移→左右轮速 (v3原版, 距离由人自行掌握) ===
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
                "owner": self.profile is not None, "score": round(self.last_score, 2),
                "smooth_cx": round(self.scx, 1), "smooth_h": round(self.sh, 1),
                "offset": round(offset, 1), "lost": self.lost}


def run_follow_loop(controller):
    while controller.running:
        controller.tick()
        time.sleep(CYCLE_S)
