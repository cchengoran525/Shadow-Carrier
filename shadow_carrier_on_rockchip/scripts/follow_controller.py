#!/usr/bin/env python3
"""follow_controller.py v4 - 差速弧线跟人 + 快速认主(颜色+体态, owner_id)"""
import os, time, json, math, urllib.request

# ========== 调参区 ==========
FW, FH = 640, 480
FCX = FW / 2

ALPHA = 0.18          # EMA基准系数(10Hz环率 τ≈0.5s); 实际按dt归一, 改环率无需重调
ALPHA_REF_DT = 0.1    # dt归一基准周期

DIR_DEADBAND = 50     # 死区: ±50px内直走
# 航向闭环 (IMU, 2026-10-04): 仅直行段生效, 压住开环走歪
YAW_K = 2.0           # dps → 轮速差 增益
YAW_CORR_MAX = 12     # 单轮最大修正量
YAW_DEAD = 1.5        # 死区(dps): 小于此值视为零偏噪声, 不修正 (治标定残差导致的持续偏航)

BASE_SPD = 130        # 基准前进速度 (100→130, 稳了再150)
MIN_SPD = 80          # 最小轮速 (60→80, 离开固件PWM<50死区)
MAX_OFFSET = 200       # 偏移饱和点

TARGET_BBOX_H = 400
MIN_BBOX_H = 150
FWD_SPD = 100

# 近距急停兜底 (2026-10-14): 恒速跟随无距离保持 → 主人/包一近会"顶着走"(撞包)
# 框高≥STOP 即停(太近), 回落到 RESUME 再走(滞回防抖)。
# ⚠️ 占位阈值, 需按实际机位/tilt 标定一次(框高随 tilt 变化; demos tilt=112)。
STOP_BBOX_H = 360      # 框高≥此值 = 太近 → STOP
RESUME_BBOX_H = 320    # 回落至此值以下 → 恢复前进

H_CHANGE_LIMIT = 0.3

CYCLE_S = 0.1         # 名义控制周期(仅实机路径参考; 判定逻辑按实测dt计时, 不依赖环率)
# 以下阈值一律用"秒": 实机10Hz / 仿真0.3s / 任意环率下行为一致
LOST_TIMEOUT_S = 2.4      # 连续丢失多久 → wait_owner / lost_stop
SEARCH_BACK_S = 1.2       # 丢人后小幅后退搜索时长
LOW_STREAK_S = 3.2        # 连续低分多久 → 清跟踪器(逃逸错锁)
BACK_SPD = 85
BACK_MS = 0.25
# 丢人搜索: 找不到人时先小幅后退找人(配合云台搜索), 而不是只靠云台摇摆
SEARCH_BACK_SPD = 70      # 后退速度(比 BACK_SPD 慢, 稳)

DET_API = "http://127.0.0.1:8080/api/detections"
# ===== 跨进程主人框发布 (HRI A1.2 依赖, 2026-10-04) =====
# 仅在模板匹配成功时写; 失配/丢失不写 → 文件自然过期, 消费方回退
OWNER_JSON = "/dev/shm/owner.json"
OWNER_JSON_MIN_INTERVAL = 0.2   # 最快发布间隔(秒)
_last_publish = 0.0

def _publish_owner(box):
    """落盘主人框: {ts, bbox:[x1,y1,x2,y2]}, 原子替换避免消费方读到半截"""
    global _last_publish
    now = time.time()
    if now - _last_publish < OWNER_JSON_MIN_INTERVAL:
        return
    _last_publish = now
    try:
        tmp = OWNER_JSON + ".tmp"
        with open(tmp, "w") as f:
            json.dump({"ts": round(now, 3),
                       "bbox": [round(float(box["x1"]), 1), round(float(box["y1"]), 1),
                                round(float(box["x2"]), 1), round(float(box["y2"]), 1)]}, f)
        os.replace(tmp, OWNER_JSON)
    except Exception as e:
        print(f"[follow] owner.json publish error: {e}")
# ============================
# ============================

class FollowController:
    def __init__(self, send_cmd_fn, bearing_fn=None, person_provider=None,
                 yaw_rate_fn=None, dist_fn=None):
        # bearing_fn: [云台]线提供的"主人世界方位角(度)"只读回调。
        # 非None时转向误差用它(云台解耦), 否则退回原始bbox像素偏差。
        # person_provider: 仿真检测输入; 实机默认仍走 owner_id 认主链路。
        # yaw_rate_fn: [云台]线IMU回调(度/秒, 逆时针为正); 直行段用于航向闭环纠偏。
        self.bearing_fn = bearing_fn
        self.person_provider = person_provider
        self.yaw_rate_fn = yaw_rate_fn
        # dist_fn: 可选"人到车距离(米)"回调([世界]提供后接线); None=恒速
        self.dist_fn = dist_fn
        self._last_t = None      # EMA/计时共用: 实测dt
        self._lost_s = 0.0       # 连续丢失秒数
        self._low_s = 0.0        # 连续低分秒数
        self.send_cmd = send_cmd_fn
        self.scx = FCX
        self.scy = FH / 2
        self.sh = 0
        self.lost = 0
        self.paused = False
        self.running = False
        self.last_turn_dir = None
        self.backing = False
        self.too_close = False   # 近距急停兜底状态(见 STOP_BBOX_H)
        self.profile = None      # 主人模板 (OwnerProfile)
        self.last_score = 0.0
        self.tracker = None      # 单目标α-β跟踪器(认主成功后创建)
        self.low_streak = 0      # 连续低分/丢失拍数(逃逸错锁用)

    def start(self):
        """启动跟随; 仿真注入目标检测，实机必须通过 owner_id 认主。"""
        self.running = True; self.lost = 0
        self.last_turn_dir = None; self.backing = False
        if self.person_provider is not None:
            self.profile = "simulated-owner"
            print("[follow] simulated owner acquired from injected detector")
            return True
        # 快速认主: 失败重试一次; 仍失败 → 拒绝跟随(宁笨勿邪的完整含义:
        # 没有可信模板就不跟任何人, 降级跟最大=跟随机路人是被禁止的)
        try:
            import owner_id
            p = owner_id.enroll(log=print)
            if p is None:
                print("[follow] 认主失败, 重试一次...")
                p = owner_id.enroll(log=print)
            self.profile = p
        except Exception as e:
            self.profile = None
            print(f"[follow] owner module error ({e})")
        if self.profile is None:
            self.running = False
            self.send_cmd("STOP")
            print("[follow] 认主失败x2 → 拒绝进入跟随 (不跟陌生人)")
            return False
        try:
            import owner_id
            owner_id.start_ble_heartbeat(log=print)  # 在场心跳(BLE+热点双路)
        except Exception:
            pass
        print("[follow] started")
        return True

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
        """Use an injected detector when present; otherwise require the enrolled owner profile."""
        if self.person_provider is not None:
            try:
                return self.person_provider()
            except Exception as e:
                print(f"[follow] injected detector error: {e}")
                return None
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
                if self._lost_s > LOST_TIMEOUT_S and self.tracker is not None:
                    self.tracker.reset()
                box, score = owner_id.select_target(
                    self.profile, img, dets, tracker=self.tracker)
                self.last_score = score
                if box is None:
                    self.low_streak += 1
                    if self.tracker is not None:
                        self.tracker.coast()
                        if self.low_streak >= 1 and self._low_s >= LOW_STREAK_S:
                            self.tracker.reset()
                            self.low_streak = 0
                            self._low_s = 0.0
                            print("[follow] re-acquire: tracker reset")
                    # 机主原则(2026-08-28 定稿): 模板失配 = 画面里没有可信的主人
                    # → 返回None走丢失流程(wait_owner/lost_stop), 绝不转投路人
                    return None
                _publish_owner(box)   # 发布给 HRI 等消费方(纯新增)
                cx = (box["x1"] + box["x2"]) / 2
                cy = (box["y1"] + box["y2"]) / 2
                if self.tracker is None:
                    self.tracker = owner_id.TargetTracker()
                self.tracker.update(cx, cy)
                if score < owner_id.SCORE_MIN - 0.05:
                    self.low_streak += 1
                    self._low_s += _dt
                    if self._low_s >= LOW_STREAK_S:
                        self.tracker.reset()
                        self.low_streak = 0
                        self._low_s = 0.0
                        print("[follow] re-acquire: low-score streak")
                else:
                    self.low_streak = 0
                    self._low_s = 0.0
                return {"cx": cx, "h": box["y2"] - box["y1"], "cy": cy}
            except Exception as e:
                print(f"[follow] owner select error: {e}")
                # 机主原则: 模块异常时同样不降级跟陌生人, 走丢失流程
                self.profile = None
                return None
        return None

    def _diff(self, left, right):
        """发送差速命令"""
        self.send_cmd(f"DIFF L{left} R{right}")

    def tick(self):
        if not self.running: return None, None, {"state": "stopped"}
        if self.paused: return None, None, {"state": "paused"}

        # 实测 dt (与环率解耦: 实机10Hz / 仿真0.3s 行为一致)
        _now = time.monotonic()
        _dt = (_now - self._last_t) if self._last_t else 0.0
        self._last_t = _now
        _dt = max(0.0, min(0.5, _dt))

        # 回退中
        if self.backing:
            self.send_cmd("STOP")
            self.backing = False
            self.last_turn_dir = None
            return "STOP", 0, {"state": "back_done"}

        person = self._fetch_person()

        if person is None:
            self.lost += 1
            self._lost_s += _dt
            # 丢人: 先小幅后退找人(配合云台搜索), 不是只靠云台摇摆
            # 首拍确认(防单帧闪烁), 随后 SEARCH_BACK_S 秒内小幅后退
            if self.lost == 1:
                return None, None, {"state": "lost", "count": self.lost}
            if self._lost_s <= SEARCH_BACK_S:
                self.send_cmd(f"MOVE B {SEARCH_BACK_SPD}")
                return "B", SEARCH_BACK_SPD, {"state": "search_back",
                                              "lost": self.lost}
            if self._lost_s >= LOST_TIMEOUT_S:
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
            return None, None, {"state": "lost", "count": self.lost}

        self.lost = 0
        self._lost_s = 0.0
        raw_cx, raw_h = person["cx"], person["h"]

        # EMA 平滑 (dt归一: 环率变化不改变等效平滑时窗)
        _a = 1.0 - (1.0 - ALPHA) ** (max(0.02, _dt) / ALPHA_REF_DT)
        if self.sh == 0:
            self.scx, self.scy, self.sh = raw_cx, person["cy"], raw_h
        else:
            h_change = abs(raw_h - self.sh) / self.sh if self.sh > 0 else 0
            if h_change < H_CHANGE_LIMIT:
                self.scx = _a * raw_cx + (1 - _a) * self.scx
                self.scy = _a * person["cy"] + (1 - _a) * self.scy
                self.sh  = _a * raw_h + (1 - _a) * self.sh

        offset = self.scx - FCX
        # 云台解耦: 世界方位角→等效像素偏差 (FX≈508 来自 state/calib/params.py)
        # 云台边缘保持会把原始bbox误差钳在~108px, 这里把它补回来
        if self.bearing_fn is not None:
            th = self.bearing_fn()
            if th is not None:
                offset = 508.0 * math.tan(math.radians(th))
        abs_off = abs(offset)

        # === 近距急停兜底: 框太大=太近 → 停(不顶人/包); 滞回恢复 ===
        if self.sh >= STOP_BBOX_H:
            self.too_close = True
        elif self.sh <= RESUME_BBOX_H:
            self.too_close = False
        if self.too_close:
            self.send_cmd("STOP")
            return "STOP", 0, self._info(offset)

        # === 差速映射: 偏移→左右轮速 (v3原版 + 可选距离匹配) ===
        # 距离匹配(进阶): 仅当 dist_fn 提供新鲜距离时微调基准速度; 默认恒速
        fwd = BASE_SPD
        if self.dist_fn is not None:
            try:
                _d = self.dist_fn()
            except Exception:
                _d = None
            if _d is not None and 0.4 < _d < 8.0:
                # 目标间距1.2m; 人远→快、人近→慢, 保守±30
                fwd = int(max(MIN_SPD, min(160, BASE_SPD + (_d - 1.2) * 30)))
        if abs_off <= DIR_DEADBAND:
            # 直走 + 航向闭环(IMU): 压住开环走歪
            left = right = fwd
            if self.yaw_rate_fn is not None:
                yr = self.yaw_rate_fn()
                if yr is not None and abs(yr) >= YAW_DEAD:
                    # yr>0 = 逆时针(左漂) → 需右修: 左轮快/右轮慢
                    corr = max(-YAW_CORR_MAX, min(YAW_CORR_MAX, YAW_K * yr))
                    hi = fwd + YAW_CORR_MAX
                    left = int(max(20, min(hi, fwd + corr)))
                    right = int(max(20, min(hi, fwd - corr)))
            self.last_turn_dir = None
        else:
            # 弧线: 一侧降速
            ratio = min(abs_off / MAX_OFFSET, 1.0)
            slow = int(fwd - ratio * (fwd - MIN_SPD))
            if offset < 0:
                # 人在左→左轮慢, 右轮快=左弧
                left, right = slow, fwd
            else:
                # 人在右→右轮慢, 左轮快=右弧
                left, right = fwd, slow
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
