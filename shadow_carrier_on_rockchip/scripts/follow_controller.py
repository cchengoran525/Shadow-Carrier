#!/usr/bin/env python3
"""follow_controller.py v5 - 差速弧线跟人 + 底盘转向通道(原地转/掉头搜索) + 快速认主

v5 (2026-10-16, 云台冻结期): 新增"底盘转向通道"——**不替代弧线**, 只作为独立状态,
在"人绕到侧后 / 正在走出画面 / 已丢失"三种情形被调用; 由 TURN_ENABLED 逐步启用。
"""
import os, sys, time, json, math, urllib.request

# ---- 可选依赖: state/ 线的解码与滤波 (缺失时降级, 不影响原有弧线跟随) ----
_st = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "state")
if _st not in sys.path:
    sys.path.insert(0, _st)
try:                                       # 像素→方位角(去畸变), 权威实现在 state/decoder.py
    from decoder import pixel_to_bearing_offset as _pixel_to_bearing   # noqa: E402
except Exception:
    _pixel_to_bearing = None
try:                                       # 极坐标 CV 卡尔曼 (θ, θ̇, r, ṙ)
    from tracker import PolarCVKF as _PolarCVKF                        # noqa: E402
except Exception:
    _PolarCVKF = None

# ========== 调参区 ==========
FW, FH = 640, 480
FCX = FW / 2

ALPHA = 0.18          # EMA基准系数(10Hz环率 τ≈0.5s); 实际按dt归一, 改环率无需重调
ALPHA_REF_DT = 0.1    # dt归一基准周期

DIR_DEADBAND = 50     # 死区: ±50px内直走
# 航向闭环 (IMU, 2026-10-04): 仅直行段生效, 压住开环走歪
YAW_K = 1.0           # dps → 轮速差 增益 (2.0→1.0: 全程角速度闭环后降增益防振荡; 现场可调)
YAW_CORR_MAX = 20     # 单轮最大修正量 (16→20, 全程闭环后需要一点余量)
YAW_DEAD = 1.5        # 死区(dps): 小于此值视为零偏噪声, 不修正 (治标定残差导致的持续偏航)
YAW_WHEEL_MIN = 120   # 角速度修正时单轮下限(防重载内轮掉到死区停转)
# 弧线段 IMU 角速度闭环 (2026-10-21): 旧版只在 |offset|<=DIR_DEADBAND 用 IMU, 跟随偏移时
# 弧线段无反馈 → 纯 P 遇轮速不一致/重载会画弧/蛇形。改为全程: w_des(期望转速) - yaw_rate。
W_DES_GAIN = 0.18     # dps/px: 超出死区后每 px 的期望转向角速度
W_DES_MAX = 35.0      # dps: 期望转向角速度上限

BASE_SPD = 245        # 基准前进速度 (220→245, 2026-10-21 重载档: 车重↑需更大基础力矩)
MIN_SPD = 155         # 最小轮速 (120→155, 重载下内轮低于此会带不动/转不动)
MAX_OFFSET = 200       # 偏移饱和点

TARGET_BBOX_H = 400
MIN_BBOX_H = 150
FWD_SPD = 100

# 近距"距离保持" (2026-10-14 v2): 首选真实距离(dist_fn); 未就绪时用框高兜底。
# ⚠️ tilt=112 下框高随距离"压缩", 阈值需实测微调(高→太近才停, 低→正常距离误停)。
NEAR_STOP_M = 0.7      # dist_fn 给出的人-车距离 < 此值 → STOP(真实距离路径)
SLOW_BBOX_H = 340      # 框高≥此值 → 开始减速(无 dist_fn 时的兜底)
STOP_BBOX_H = 400      # 框高≥此值 → 硬停(无 dist_fn 时的兜底)
RESUME_BBOX_H = 370    # 框高回落至此值以下 → 恢复
NEAR_GATE_OFFSET = 100 # 仅当人接近正前(offset≤此px, ≈11°)才做框高急停/减速——
                       # passby 时人在侧边(框大但不是在车头)不能误停(2026-10-18 实测"转30°就停")
# ===== 速度门 (2026-10-19): |offset| 越大越慢 =====
# 依据: BASE_SPD 上调到 220(≈0.86 m/s)后制动距离 ≈ v·t_react + v²/2a ≈ 0.9m; 而"人很近但
# 不在正前"时两个近距信号都不可用 —— 框高在 ~2.2m 内饱和(1m 与 2m 同高), 脚底地面测距在
# <1.58m 已出画(见 state/range_from_box.py)。唯一可靠代理是**方位角**: 人越靠边, 车离他
# 越近, 于是把前向速度按 |offset| 线性压低(正前 11° 内不受影响, 到 30° 保留 55%)。
PIVOT_ENTER_PX = 290   # ≈30° (=FX·tan30°), 与 PIVOT_ENTER_DEG 对齐(超过它走转向通道)
V_GATE_DROP = 0.45     # 到 PIVOT_ENTER_PX 时的前向速度降幅

H_CHANGE_LIMIT = 0.3
H_CHANGE_CONFIRM_N = 3   # 高度突变持续多少拍才算"真实远近变化"(10Hz → 0.3s); 否则按单帧异常忽略

CYCLE_S = 0.1         # 名义控制周期(仅实机路径参考; 判定逻辑按实测dt计时, 不依赖环率)
# 以下阈值一律用"秒": 实机10Hz / 仿真0.3s / 任意环率下行为一致
LOST_TIMEOUT_S = 2.4      # 连续丢失多久 → wait_owner / lost_stop
SEARCH_BACK_S = 1.2       # 丢人后小幅后退搜索时长
LOW_STREAK_S = 3.2        # 连续低分多久 → 清跟踪器(逃逸错锁)

# ===== 换人确认门 (2026-10-16, passby 防吸路人) =====
# 背景: TRACK 阶段原对任何一帧匹配到的人立即认作强证据 → 主人擦身出画时,
#       同色路人单帧即可顶替锁定。改为: 非连续候选需连续帧确认后才生效。
# 连续(与跟踪器预测一致)或首次锁定 → 不延迟; 仅"换人/重获"付确认代价。
LOCK_CONFIRM_N = 3          # 非连续候选(换人/重获)需连续帧数
LOCK_CONFIRM_STRICT_N = 5   # 候选在"出画侧反方向"时更严(擦身窗口)
SWITCH_SCORE_MARGIN = 0.03  # 非连续候选起始门槛: SCORE_MIN + margin
SWITCH_MAX_PX = 200         # 与跟踪器预测中心超过此值 = 非连续(换人候选)
BACK_SPD = 110
BACK_MS = 0.25
# 丢人搜索: 找不到人时先小幅后退找人(配合云台搜索), 而不是只靠云台摇摆
SEARCH_BACK_SPD = 95      # 后退速度(重载档 70→95; 仍比 BACK_SPD 慢)

# ========== 底盘转向通道 (v5, 2026-10-16 统筹批准 v3) ==========
# 原则1: 前进跟人 = 弧线(DIFF), 上面那套映射一行不改 (2026-08-08 裁定继续生效)
#        当年"直行-坦克-直行-坦克"的根因是拿原地转**替代**弧线做前进, 不是原地转本身
# 原则2: 原地转(MOVE L/R) = 独立状态, 退出即回弧线, 不做连续混合
# 原则3: 授权分级 —— 强证据(YOLO+模板)才能**发起**转向与放行平移;
#        弱线索(低置信框/OpenCV 腿带)只能**维持**已发起的原地转, 绝不放行前进、
#        不进 KF、不写 owner.json (机主原则第0层, owner_id.py:157)
TURN_ENABLED = True      # 总开关: 2026-10-20 [运动控制] T4 passby 启用(manual 模式无副作用, follow 才生效; 回退改 False + 重启 rk-control)

PIVOT_ENTER_DEG = 30.0   # E2 兜底入口: |θ|≥此值持续 PIVOT_ENTER_S
PIVOT_ENTER_S = 0.3
PIVOT_EXIT_DEG = 15.0    # 退出口(与入口构成 2:1 迟滞; 参照 Nav2 RotationShim 45/22.5)
PIVOT_PWM = 240          # MOVE L/R 速度 (190→240, 重载档: 原地转需更大扭矩; 角速度待 pivot_calib)
# 原地转的标称角速度(°/s): 190 PWM 在仿真运动学下 ≈174°/s。实机由 scripts/pivot_calib.py
# 实测覆盖。用途: 把"角度预算"翻译成"时间预算"(盲转 60°≈0.34s; 扫描 90° 一档≈0.52s),
# 让 测试/仿真/生产 共用同一口径。改 PIVOT_PWM 或换底盘(麦轮/编码器)都要重跑标定并改这里。
PIVOT_RATE_DPS = 174.0
PIVOT_DWELL_S = 0.25     # 模式/方向最小驻留(防抖)
PIVOT_BUDGET_DEG = 180.0 # TURN 相(人可见)的角度预算(超 → 停下等待)
PIVOT_BLIND_MAX_DEG = 60.0   # BLIND 相的角度上限: 失明期 θ̇ 会冻结(无新量测), 若按陈旧
#                              角速度积分会转过头 —— 2026-10-16 仿真实测: 2s 盲转 = 190°,
#                              把刚擦身过去的人直接甩到身后另一侧 → 永远重捕获不到。
#                              改为"只转估计需要的那点", 剩下的交给小步扫描(SEARCH)。
PIVOT_BLIND_WEAK_DEG = 150.0  # 弱线索存活时的盲转额度(腿还在画面里 → 值得继续跟)
PIVOT_TIMEOUT_S = 6.0    # 单次转向时长上限
REVERSAL_MAX = 3         # 窗口内方向反转上限 → 强制回弧线(设计文档 §5.4.6 "一键回退 DIFF")
REVERSAL_WIN_S = 5.0

# E1 预测式入口: 人在画面内但**正在往外走** → 提前转 (角速度触发, 不用角度)
TTE_MAX_DEG = 30.0       # 出画角(视野 ±32.2°, 留 2° 余量)
TTE_TRIGGER_S = 0.35
TTE_MIN_DPS = 40.0       # 目标世界角速度下限(低于此视为"人没在往外走", 正常跟走永不触发)
E1_MAX_OWN_DPS = 20.0    # E1 前置门: 自身角速度超此值时不判 E1
# 理由(test_turn_pivot D 实测): 车体角速度突变时 KF 的 θ̇ 会滞后 1~2 拍, 归因项补不齐,
# "我在转"会被短时误算成"人在走"; 车体安静时才允许做预测式转向决策。
EDGE_DEG = 24.0          # 贴边记出画侧(供丢人后的定向扫描)

# 弱线索 / 盲区 / 定向扫描
WEAK_TTL_S = 3.0         # 弱线索有效期(自最后一次强确认起算; "只维持不发起")
WEAK_MIN_SCORE = 0.35    # 弱线索强度下限(各源归一化后给)
BLIND_MAX_S = 1.2        # 纯预测盲区的桥接时长(BLIND 是"桥": 转估计该转的那点就行,
#                          有目的的搜索交给 SEARCH; 角度上限见 PIVOT_BLIND_MAX_DEG)
SEARCH_STEP_DEG = 90.0   # 定向扫描每档角度
SEARCH_DWELL_S = 0.5     # 每档停顿(让检测落地)
SEARCH_MAX_DEG = 180.0   # 扫描总量上限(不做 360° 持续旋转: §12.4 "不动比动更被信任")
SEARCH_TOL_DEG = 5.0     # 到位判据

# 自身角速度 → θ 约定 的符号。θ 与弧线 offset 同源(同号), 而 MOVE L/R 与 offset 的
# 转向方向由弧线映射锁定 → 两者不会各说各话(2026-10-14 符号链审计的教训)。
# sim 内已由 test_turn_pivot.py 验证; 真机用 scripts/pivot_calib.py 一测即可定。
IMU_TO_THETA_SIGN = -1.0

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
                 yaw_rate_fn=None, dist_fn=None, yaw_deg_fn=None, obs_fn=None,
                 weak_cue_fn=None, bearing_of_fn=None):
        # bearing_fn: [云台]线提供的"主人世界方位角(度)"只读回调。
        # 非None时转向误差用它(云台解耦), 否则退回原始bbox像素偏差。
        # person_provider: 仿真检测输入; 实机默认仍走 owner_id 认主链路。
        # yaw_rate_fn: [云台]线IMU回调(度/秒, 逆时针为正); 直行段用于航向闭环纠偏。
        self.bearing_fn = bearing_fn
        self.person_provider = person_provider
        self.yaw_rate_fn = yaw_rate_fn
        # dist_fn: 可选"人到车距离(米)"回调([世界]提供后接线); None=恒速
        self.dist_fn = dist_fn
        # ---- 底盘转向通道(v5)注入: 缺失时该通道整体降级, 弧线跟随不受影响 ----
        self.yaw_deg_fn = yaw_deg_fn     # 积分航向(度), 原地转的死推算与到位判据
        self.obs_fn = obs_fn             # () -> (blocked, cm) 或 None; 超声波否决原地转
        self.weak_cue_fn = weak_cue_fn   # () -> {"dir","score","src"} 或 None; 只维持不发起
        # bearing_of_fn: 像素列 → 车体系方位角(度)。实机默认走 state/decoder(真机内参);
        # 仿真注入自己投影用的内参, 否则 CX/fx 不一致会引入常数偏置+尺度误差(实测 +3°/15%)。
        self.bearing_of_fn = bearing_of_fn
        self._last_t = None      # EMA/计时共用: 实测dt
        self._lost_s = 0.0       # 连续丢失秒数
        self._low_s = 0.0        # 连续低分秒数
        self._conf_center = None # 换人确认门: 候选中心
        self._conf_n = 0         # 换人确认门: 连续帧计数
        self._ever_locked = False  # 是否完成过首次锁定(首次不延迟)
        self._switched_this_frame = False
        self.send_cmd = send_cmd_fn
        self.scx = FCX
        self.scy = FH / 2
        self.sh = 0
        self.lost = 0
        self.paused = False
        self.running = False
        self.last_turn_dir = None
        self.backing = False
        self.too_close = False   # 近距"距离保持"状态(框高兜底)
        self.profile = None      # 主人模板 (OwnerProfile)
        self.last_score = 0.0
        self.tracker = None      # 单目标α-β跟踪器(认主成功后创建)
        self.low_streak = 0      # 连续低分/丢失拍数(逃逸错锁用)
        # ---- 底盘转向通道状态 ----
        self.phase = "TRACK"     # TRACK / TURN / BLIND / SEARCH
        self.kf = None           # PolarCVKF(θ,θ̇,r,ṙ); 强量测驱动
        self._theta = None       # 车体系方位角(度, 与弧线 offset 同号), KF 估计
        self._theta_raw = None   # 同上, 但为最近一帧的原始量测(转向决策用, 见 _theta_ctl)
        self._theta_dot = 0.0    # 相对角速度(度/秒, KF)
        self._kf_q = 1.0         # KF quality(exp(-age/1.2))
        self._last_strong_t = None   # 最后一次强确认(单调钟秒)
        self._exit_sign = None   # 贴边出画侧: "L"/"R" (丢人后定向扫描用)
        self._exit_t = 0.0
        self._e1_hits = 0        # E1 连续确认拍数
        self._edge_s = 0.0       # E2 贴边持续秒数
        self.turn_dir = None     # 当前原地转方向 "L"/"R"
        self._turn_started = 0.0
        self._turn_deg = 0.0     # 累计转角(IMU)
        self._turn_last_yaw = None
        self._dir_changes = []   # 方向反转时间戳(看门狗)
        self._fallback_until = 0.0   # 看门狗触发后的弧线强制期
        self._search_spent = 0.0     # 搜索累计转角
        self._search_hold_until = 0.0
        self._reacq_hits = 0     # 重捕获双帧确认
        self._weak_last = None   # 最近一次弱线索(黑匣子/调试)
        self._kf_t = None        # KF/降级差分 的上次量测时刻
        self._last_dir_t = 0.0   # 上次改方向时刻(最小驻留)
        self._step_deg = 0.0     # 本档已转角度(搜索分档)
        self._searched_at = 0.0  # 上次扫描用尽时刻(抑制"扫完又后退搜索")
        self._turn_domain = False   # tick(turn_only=True): 停驻域(HIDE/WAIT)只做有界转向
        self._h_jump = 0         # 框高突变确认计数(见 H_CHANGE_CONFIRM_N)
        self._w_des = 0.0        # 期望转向角速度(dps, 黑匣子/调试)

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

    def _lock_confirmed(self, cx, cy, score):
        """换人确认门(passby 防吸路人)。True=确认可用作强证据。
        连续(与跟踪器预测一致)或首次锁定 → 直接 True; 非连续候选需连续帧确认,
        且位于"出画侧反方向"时更严。确认期间不发布 owner.json、不刷新强证据。"""
        import math as _m
        import owner_id as _oid
        pred = self.tracker.predict() if self.tracker is not None else None
        if pred is not None and self.lost == 0 and \
                _m.hypot(cx - pred[0], cy - pred[1]) <= SWITCH_MAX_PX:
            self._conf_center = None; self._conf_n = 0
            self._switched_this_frame = False
            return True                      # 连续跟踪 → 正常
        if pred is None and not self._ever_locked:
            self._conf_center = None; self._conf_n = 0
            self._ever_locked = True
            self._switched_this_frame = False
            return True                      # 认主后首次锁定 → 不延迟
        need = LOCK_CONFIRM_N
        cand_dir = "R" if cx > FCX else "L"
        if self._exit_sign is not None and cand_dir != self._exit_sign:
            need = LOCK_CONFIRM_STRICT_N     # 与主人出画侧相反 → 更可能是路人
        if (self._conf_center is not None and
                _m.hypot(cx - self._conf_center[0], cy - self._conf_center[1]) <= SWITCH_MAX_PX):
            self._conf_n += 1
        else:
            self._conf_center = (cx, cy); self._conf_n = 1
        if score < _oid.SCORE_MIN + SWITCH_SCORE_MARGIN or self._conf_n < need:
            self._switched_this_frame = False
            print(f"[follow] 换人待确认 {self._conf_n}/{need} score={score:.2f} dir={cand_dir}")
            return False
        self._conf_center = None; self._conf_n = 0
        self._ever_locked = True
        self._switched_this_frame = True   # 换人确认成功 → 调用方重置跟踪器
        return True

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
                    self._conf_center = None; self._conf_n = 0   # 无候选 → 确认链断开
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
                cx = (box["x1"] + box["x2"]) / 2
                cy = (box["y1"] + box["y2"]) / 2
                # 换人确认门: 未确认 → 本拍不作为强证据(不发布/不刷新/不进KF)
                if not self._lock_confirmed(cx, cy, score):
                    return None
                if self._switched_this_frame and self.tracker is not None:
                    self.tracker.reset()   # 换人成功: 旧速度无意义, 清掉防过冲
                _publish_owner(box)   # 发布给 HRI 等消费方(纯新增)
                if self.tracker is None:
                    self.tracker = owner_id.TargetTracker()
                self.tracker.update(cx, cy)
                # 注(2026-10-16): 原此处有一段 `score < SCORE_MIN-0.05` 的低分streak分支,
                # 恒不可达(select_target 已保证 score>=SCORE_MIN), 且其内 `_dt` 是未定义名
                # (阈值一改就 NameError) → 直接删除, 保留下面的正常返回。
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

    # ==================== 底盘转向通道 (v5) ====================
    # 只在上面的弧线映射之外新增状态机; 弧线路径一行未动。
    def _bearing_of(self, cx):
        """像素列 → 车体系方位角(度)。与弧线 offset=cx-FCX 同号约定
        (左右物理含义由弧线映射锁定, 本通道不另立一套, 避免符号链分裂)。"""
        if self.bearing_of_fn is not None:
            return self.bearing_of_fn(cx)
        if _pixel_to_bearing is not None:
            return _pixel_to_bearing(cx)
        return math.degrees(math.atan((cx - FCX) / 508.2))   # 降级: 无 state/decoder 时

    def _yaw_deg(self):
        if self.yaw_deg_fn is None:
            return None
        try:
            v = self.yaw_deg_fn()
            return None if v is None else float(v)
        except Exception:
            return None

    def _yaw_rate(self):
        if self.yaw_rate_fn is None:
            return 0.0
        try:
            v = self.yaw_rate_fn()
            return 0.0 if v is None else float(v)
        except Exception:
            return 0.0

    def _rate_world(self):
        """目标世界角速度(θ约定, °/s) = 相对 θ̇(KF) + 自身角速度归算。
        归因是必须的: 不扣自身转速, "我在转身"就会被误判成"人在往外走"
        —— 正是 2026-08 坦克转向摇摆的根因之一。"""
        return self._theta_dot + IMU_TO_THETA_SIGN * self._yaw_rate()

    def _lost_since(self, now):
        return 0.0 if self._last_strong_t is None else (now - self._last_strong_t)

    def _obs_blocked(self):
        """超声波否决: 原地转虽不平移, 受阻时也不继续拧(给人通过的余量)"""
        if self.obs_fn is None:
            return False
        try:
            r = self.obs_fn()
            return bool(r and r[0])
        except Exception:
            return False

    def _weak_ok(self, now):
        """弱线索: 只维持不发起(须近期有强确认) + 不进 KF/owner.json + 时效"""
        if self.weak_cue_fn is None or self.phase not in ("TURN", "BLIND"):
            return None
        if self._last_strong_t is None or (now - self._last_strong_t) > WEAK_TTL_S:
            return None
        try:
            c = self.weak_cue_fn()
        except Exception:
            return None
        if not c or float(c.get("score", 0.0)) < WEAK_MIN_SCORE:
            return None
        self._weak_last = c
        return c

    def _kf_feed(self, person, now):
        """强量测 → θ 通道。r 用框高粗估(只影响 R 的 r 通道, 与 θ̇ 解耦);
        本通道只服务转向, 不接距离环。"""
        theta = self._bearing_of(person["cx"])
        self._theta_raw = theta          # 转向决策用原始量测, 见 _theta_ctl
        h = max(20.0, float(person["h"]))
        r = max(0.3, 492.7 * 1.0 / h)
        if _PolarCVKF is None:                      # 降级: 无 numpy/tracker → 差分+EMA
            if self._theta is not None and self._kf_t is not None:
                _dtk = now - self._kf_t
                if _dtk > 0.02:
                    self._theta_dot = 0.5 * ((theta - self._theta) / _dtk) + 0.5 * self._theta_dot
            self._theta, self._kf_t, self._kf_q = theta, now, 1.0
            return
        if self.kf is None:
            self.kf = _PolarCVKF(now, theta, r)
            # 人的角加速度比模块默认草案(8°/s²)大(转身/绕行) → 本实例上调过程噪声;
            # 只改实例属性, 不动 state/tracker.py 的默认值与它的自测基线
            self.kf.SIG_A_TH = 40.0
            self._theta, self._kf_t, self._kf_q = theta, now, 1.0
            return
        self.kf.update(now, theta, r)
        st = self.kf.state(now)
        self._theta, self._theta_dot = st["theta"], st["theta_dot"]
        self._kf_q, self._kf_t = st["quality"], now

    def _theta_ctl(self):
        """转向决策用的方位角: 优先**原始量测**。

        理由(2026-10-16 仿真实测): 原地转 ~100°/s 时每拍 θ 变 ~10°, 远超 KF 的过程噪声,
        卡方硬门会把量测整帧拒掉 → θ 冻在转弯前的值(实测冻在 31.6° 而几何真值已回 0.7°),
        车就会朝错方向一路转出去。KF 的价值在"入口预测"的 θ̇ —— 而 E1 有"自身需安静"
        的前置门, 那时估计是可用的; 决策本身用原始量测更快更准(10Hz, 无滤波滞后)。"""
        return self._theta_raw if self._theta_raw is not None else self._theta

    def _dir_of_theta(self):
        """θ 与弧线 offset 同号 → 与弧线同一套转向语义:
        弧线里 offset>0 取"左轮快右轮慢"(右转), 对应原地转就是 MOVE R。
        故 offset>0 发 R。由 test_turn_pivot.py 的收敛断言锁定。"""
        return "R" if (self._theta_ctl() or 0.0) > 0 else "L"

    def _check_e1(self):
        """E1 预测式: TTE<触发值 且 目标世界角速度够大, 连续 2 帧确认。
        前置门: 自身在大幅旋转时不判(θ̇ 滞后会把自身旋转误算成人在走)。"""
        th = self._theta_ctl()
        if th is None:
            return False
        if abs(self._yaw_rate()) > E1_MAX_OWN_DPS:
            self._e1_hits = 0
            return False
        rate = self._rate_world()
        if abs(rate) < TTE_MIN_DPS:
            self._e1_hits = 0
            return False
        tte = (TTE_MAX_DEG - abs(th)) / abs(rate)
        self._e1_hits = self._e1_hits + 1 if tte < TTE_TRIGGER_S else 0
        return self._e1_hits >= 2

    def _check_e2(self, dt):
        """E2 兜底式: 贴到视野边缘并保持 PIVOT_ENTER_S"""
        th = self._theta_ctl()
        if th is None:
            return False
        if abs(th) >= PIVOT_ENTER_DEG:
            self._edge_s += dt
        else:
            self._edge_s = 0.0
        return self._edge_s >= PIVOT_ENTER_S

    def _set_turn_dir(self, d, now):
        """改方向 + 最小驻留 + 反转看门狗(设计文档 §5.4.6 的"一键回退 DIFF")"""
        if d not in ("L", "R") or d == self.turn_dir:
            return
        if self.turn_dir is not None and (now - self._last_dir_t) < PIVOT_DWELL_S:
            return                                   # 驻留未满, 不改
        if self.turn_dir is not None:
            self._dir_changes.append(now)
            self._dir_changes = [t for t in self._dir_changes
                                 if now - t <= REVERSAL_WIN_S]
            if len(self._dir_changes) > REVERSAL_MAX:
                self._fallback_until = now + REVERSAL_WIN_S
                self._dir_changes = []
                print("[follow] 方向反转超限 → 强制回弧线(一键回退)")
        self.turn_dir = d
        self._last_dir_t = now

    def _enter_turn(self, now, d):
        self.phase = "TURN" if self._last_strong_t and (now - self._last_strong_t) < 0.05 else "BLIND"
        self._turn_started = now
        self._turn_deg = 0.0
        self._step_deg = 0.0
        self._turn_last_yaw = self._yaw_deg()
        # 2026-10-19 修: 原来把 _last_dir_t 归零(等价"驻留早已满足") → 进入后的**下一拍**
        # 就能立刻翻向(实测 0.1s 一翻)。改为: 先清 turn_dir 让初次设定生效, 驻留从进入时刻起算。
        self.turn_dir = None
        self._set_turn_dir(d, now)
        print(f"[follow] 转向通道进入 {self.phase} dir={d} "
              f"theta={None if self._theta is None else round(self._theta, 1)}°")

    def _phase_track(self, now, why):
        self.phase = "TRACK"
        self._e1_hits = self._edge_s = 0.0
        self._turn_deg = self._step_deg = 0.0
        self._reacq_hits = 0
        self._exit_sign = None      # 出画侧用毕作废, 否则转完会立刻重新进 BLIND
        print(f"[follow] 转向通道退出 → 弧线 ({why})")

    def _acc_turn_deg(self):
        """用 IMU 积分航向累计转角(原地转的死推; 单次 ≤3s 漂移 <0.2°)"""
        y = self._yaw_deg()
        if y is None:
            return
        if self._turn_last_yaw is not None:
            d = abs(y - self._turn_last_yaw)
            self._turn_deg += d
            self._step_deg += d
        self._turn_last_yaw = y

    def _pivot_cmd(self, now, dt, extra):
        """原地转一拍: 逐拍重发(靠固件 450ms 看门狗兜底)。"""
        self._acc_turn_deg()
        # 角度上限只归 TURN 相管; BLIND 相的上限由 _steer 判(额度用尽要转 SEARCH 而不是
        # 回弧线, 见 PIVOT_BLIND_MAX_DEG 的注释), 这里只留超时与超声波否决。
        over = ((self.phase == "TURN" and self._turn_deg >= PIVOT_BUDGET_DEG)
                or (now - self._turn_started) > PIVOT_TIMEOUT_S)
        if over or self._obs_blocked():
            if self.phase in ("BLIND", "SEARCH"):
                self._searched_at = now      # 转完仍没找到 → 原地等待, 不再后退搜索
            _spent = self._turn_deg
            print(f"[follow] 原地转结束 ({'预算/超时' if over else '超声波否决'}) "
                  f"累计{_spent:.0f}°")
            self._phase_track(now, "budget/obs")
            self.send_cmd("STOP")
            return "STOP", 0, {**self._info(None), **extra, "state": "turn_end",
                               "turn_deg": round(_spent, 1)}
        info = {**self._info(None), **extra, "phase": self.phase,
                "turn_dir": self.turn_dir, "turn_deg": round(self._turn_deg, 1)}
        self.send_cmd(f"MOVE {self.turn_dir} {PIVOT_PWM}")
        return "MOVE", f"{self.turn_dir}{PIVOT_PWM}", info

    def _enter_search(self, now):
        """定向扫描: 朝出画侧 90° 一档(不做 360° 持续旋转)"""
        self.phase = "SEARCH"
        self._search_spent = 0.0
        self._step_deg = self._turn_deg = 0.0
        self._turn_last_yaw = self._yaw_deg()
        self._search_hold_until = 0.0
        self._set_turn_dir(self._exit_sign or self.turn_dir or "L", now)
        print(f"[follow] 定向扫描开始 dir={self.turn_dir} 出画侧={self._exit_sign}")

    def _search_tick(self, now, dt, extra):
        if self._search_spent >= SEARCH_MAX_DEG:
            self._searched_at = now            # 标记"刚扫完", 让原丢失流程不再后退搜索
            return None                       # 扫描用尽 → 交回原有等待/丢失流程
        if now < self._search_hold_until:     # 每档停顿: 让检测落地
            self.send_cmd("STOP")
            return "STOP", 0, {**self._info(None), **extra, "state": "search_hold"}
        self._acc_turn_deg()
        step_goal = min(SEARCH_STEP_DEG, SEARCH_MAX_DEG - self._search_spent)
        if self._step_deg >= step_goal:       # 本档走完 → 停顿采样
            self._search_spent += self._step_deg
            self._step_deg = 0.0
            self._search_hold_until = now + SEARCH_DWELL_S
            self.send_cmd("STOP")
            return "STOP", 0, {**self._info(None), **extra, "state": "search_hold",
                               "search_spent": round(self._search_spent, 1)}
        if self._obs_blocked():
            self.send_cmd("STOP")
            return "STOP", 0, {**self._info(None), **extra, "state": "search_blocked"}
        self.send_cmd(f"MOVE {self.turn_dir} {PIVOT_PWM}")
        return "MOVE", f"{self.turn_dir}{PIVOT_PWM}", {
            **self._info(None), **extra, "phase": "SEARCH",
            "turn_dir": self.turn_dir, "search_spent": round(self._search_spent, 1)}

    def _steer(self, person, dt, now, turn_domain=False):
        """转向通道一拍。返回 None = 交回原有弧线/丢失流程(默认路径不变)。
        turn_domain=True: "停驻域"(HIDE/WAIT 车已停好)只做**有界转向**, 不进搜索扫描——
        社会语义上停着的车不该原地扫 180°, 但"人从侧方过来递物"时必须转头接人。"""
        self._turn_domain = turn_domain
        if now < self._fallback_until:
            return None                                  # 看门狗强制期: 只用弧线

        if person is not None:
            self._last_strong_t = now
            self._kf_feed(person, now)
            self._reacq_hits = min(self._reacq_hits + 1, 5)
            _th_raw = self._theta_ctl()
            if _th_raw is not None and abs(_th_raw) >= EDGE_DEG:
                self._exit_sign = self._dir_of_theta()   # 贴边→记出画侧(供搜索)
                self._exit_t = now
        else:
            self._reacq_hits = 0

        # ---- 已在原地转 ----
        if self.phase == "TURN":
            if person is not None:
                # 退出判据用**原始量测**: 原地转时 KF 的卡方门会整帧拒绝 → θ 冻结
                # (2026-10-19 实测: 174°/s 下 KF θ 冻在 31°, 真值已回 10° → 永不退出)
                _th_exit = self._theta_ctl()
                if (_th_exit is not None and abs(_th_exit) <= PIVOT_EXIT_DEG
                        and (now - self._turn_started) >= PIVOT_DWELL_S):
                    self._phase_track(now, "aligned")    # 回锥内 → 本拍即交回弧线
                    return None
                self._set_turn_dir(self._dir_of_theta(), now)
                return self._pivot_cmd(now, dt, {"state": "turn"})
            print("[follow] 强检测丢失 → BLIND(按预测继续转)")
            self.phase = "BLIND"

        # ---- 盲区 / 定向扫描 ----
        if self.phase in ("BLIND", "SEARCH"):
            if person is not None and self._reacq_hits >= 2:
                self._phase_track(now, "reacquire")
                if self.tracker is not None:
                    self.tracker.reset()                 # 久别重逢: 旧预测已失效
                return None
            if self.phase == "BLIND":
                w = self._weak_ok(now)
                # 弱线索还在(腿在画面里) → 额度大些; 纯预测 → 只转"估计需要的那点"
                cap = PIVOT_BLIND_WEAK_DEG if w is not None else PIVOT_BLIND_MAX_DEG
                bridged = (self._lost_since(now) <= BLIND_MAX_S) or (w is not None)
                if bridged and self._turn_deg < cap:
                    if w and w.get("dir") in ("L", "R"):
                        self._set_turn_dir(w["dir"], now)   # 弱线索只改方向, 不放行前进
                    return self._pivot_cmd(now, dt, {
                        "state": "blind",
                        "weak": None if not self._weak_last else self._weak_last.get("src")})
                # 盲转额度用尽 / 超时 / 弱线索过期 → 转有目的的小步扫描
                if self._turn_domain:
                    # 停驻域: 只做人从侧方过来时的有界转头, 不启动 180° 扫描
                    self._phase_track(now, "turn-only idle")
                    self.send_cmd("STOP")
                    return "STOP", 0, {**self._info(None), "state": "turn_idle"}
                self._enter_search(now)
            return self._search_tick(now, dt, {"state": "search"})

        # ---- 新入口 (只在 TRACK) ----
        if self.phase == "TRACK":
            if person is not None:
                if self._check_e1() or self._check_e2(dt):
                    self._enter_turn(now, self._dir_of_theta())
                    return self._pivot_cmd(now, dt, {"state": "turn"})
            elif (self._exit_sign is not None and (now - self._exit_t) <= 2.0
                    and self._lost_since(now) >= 0.4):
                # 侧向出画 + 强检测已丢 → 直接进 BLIND(不再后退搜索: 人在旁边/身后时后退是反效果)
                self._enter_turn(now, self._exit_sign)
                return self._pivot_cmd(now, dt, {"state": "blind"})
        return None

    # ==========================================================

    def tick(self, turn_only=False):
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

        # ===== 底盘转向通道 (v5) =====
        # TURN_ENABLED=False 时整段不执行 → 行为与 v4 逐字节一致(逐步启用/可回退)
        if TURN_ENABLED:
            _out = self._steer(person, _dt, _now, turn_domain=turn_only)
            if _out is not None:
                return _out
        if turn_only:
            # 停驻域(HIDE/WAIT): 转向通道没话说就什么都不做 —— HIDE 自己的驶停逻辑接管
            return None, None, {"state": "turn_idle", "phase": self.phase}

        if person is None:
            self.lost += 1
            self._lost_s += _dt
            # 丢人: 先小幅后退找人(配合云台搜索), 不是只靠云台摇摆
            # 首拍 + 0.4s 确认(防近距离"半身出画"闪烁就后退 → 治"前后前后")
            if self.lost == 1 or self._lost_s < 0.4:
                return None, None, {"state": "lost", "count": self.lost}
            # 刚扫完 180° 又落回丢失流程 → 不再后退搜索(人可能就在旁边/身后, 后退是反效果)
            _just_searched = TURN_ENABLED and (_now - self._searched_at) < 10.0
            if self._lost_s <= SEARCH_BACK_S and not _just_searched:
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
                self._h_jump = 0
                self.scx = _a * raw_cx + (1 - _a) * self.scx
                self.scy = _a * person["cy"] + (1 - _a) * self.scy
                self.sh  = _a * raw_h + (1 - _a) * self.sh
            else:
                # 高度突变门(治弯腰/蹲下的单帧异常)。2026-10-19 审查修的 bug:
                # 原实现直接跳过整帧更新 → 而"人走近"是**持续**的高度变化(300→450 恒 >30%),
                # sh 不动 → 条件永远成立 → EMA **永久冻结**(scx/sh 再不更新, 车保持旧方向)。
                # 改为: 连续 H_CHANGE_CONFIRM_N 拍都在新水平附近 → 视为真实远近变化, 重新播种。
                self._h_jump = getattr(self, "_h_jump", 0) + 1
                if self._h_jump >= H_CHANGE_CONFIRM_N:
                    self.scx, self.scy, self.sh = raw_cx, person["cy"], raw_h
                    self._h_jump = 0
                    print(f"[follow] 框高持续变化 → 重新播种 scx={raw_cx:.0f} h={raw_h:.0f}")

        offset = self.scx - FCX
        # 云台解耦: 世界方位角→等效像素偏差 (FX≈508 来自 state/calib/params.py)
        # 云台边缘保持会把原始bbox误差钳在~108px, 这里把它补回来
        if self.bearing_fn is not None:
            th = self.bearing_fn()
            if th is not None:
                offset = 508.0 * math.tan(math.radians(th))
        abs_off = abs(offset)

        # === 近距"距离保持" (降速 + 太近停) ===
        # 首选真实距离(dist_fn); 未就绪时用框高兜底(⚠️ tilt=112 下阈值需实测)
        _d = None
        if self.dist_fn is not None:
            try:
                _d = self.dist_fn()
            except Exception:
                _d = None
        if _d is not None and _d < NEAR_STOP_M:            # 真实距离太近 → 停
            self.send_cmd("STOP")
            return "STOP", 0, self._info(offset)
        if _d is None and abs_off <= NEAR_GATE_OFFSET:      # 无距离源 + 人在正前 → 框高兜底
            if self.sh >= STOP_BBOX_H:
                self.too_close = True
            elif self.sh <= RESUME_BBOX_H:
                self.too_close = False
            if self.too_close:
                self.send_cmd("STOP")
                return "STOP", 0, self._info(offset)

        # === 差速映射: 偏移→左右轮速 (v3原版 + 距离匹配/框高减速) ===
        fwd = BASE_SPD
        if _d is not None and 0.4 < _d < 8.0:
            # 目标间距1.2m; 人远→快、人近→慢, 保守±30
            fwd = int(max(MIN_SPD, min(255, BASE_SPD + (_d - 1.2) * 30)))
        elif _d is None and abs_off <= NEAR_GATE_OFFSET and self.sh > SLOW_BBOX_H:
            # 框高减速兜底: 越近越慢(到 STOP_BBOX_H 已硬停)
            frac = min(1.0, (self.sh - SLOW_BBOX_H) / max(1.0, STOP_BBOX_H - SLOW_BBOX_H))
            fwd = int(max(MIN_SPD, BASE_SPD - frac * (BASE_SPD - MIN_SPD) * 0.6))
        # 速度门: 人越靠边越慢(正前 11° 内不生效, 30° 时保留 55%) —— 近距 off-axis 的唯一
        # 可靠代理(见 PIVOT_ENTER_PX 上方的依据); 正前直行段完全不受影响。
        if abs_off > NEAR_GATE_OFFSET:
            _vg = min(1.0, (abs_off - NEAR_GATE_OFFSET)
                      / max(1.0, float(PIVOT_ENTER_PX - NEAR_GATE_OFFSET)))
            fwd = int(max(MIN_SPD, round(fwd * (1.0 - V_GATE_DROP * _vg))))
        # ---- 基础差速(前馈): 正前直行 / 偏移→弧线 ----
        if abs_off <= DIR_DEADBAND:
            left = right = fwd
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

        # ---- IMU 角速度闭环 (全程, 不再只限正前死区) ----
        # w_des: 人偏右(offset>0) → 右转(CW, yaw_rate 为负); 正前 → 0(保直)
        # e = w_des - yaw_rate; corr>0 = 需要更多 CCW → 右轮快/左轮慢
        self._w_des = 0.0
        if self.yaw_rate_fn is not None:
            yr = self.yaw_rate_fn()
            if yr is not None:
                w_des = 0.0
                if abs_off > DIR_DEADBAND:
                    sgn = 1.0 if offset > 0 else -1.0
                    w_des = -W_DES_GAIN * sgn * (abs_off - DIR_DEADBAND)
                    w_des = max(-W_DES_MAX, min(W_DES_MAX, w_des))
                self._w_des = w_des
                e = w_des - yr
                if abs(e) >= YAW_DEAD:
                    corr = max(-YAW_CORR_MAX, min(YAW_CORR_MAX, YAW_K * e))
                    left = int(max(YAW_WHEEL_MIN, min(255, left - corr)))
                    right = int(max(YAW_WHEEL_MIN, min(255, right + corr)))

        self._diff(left, right)
        return "DIFF", f"L{left}R{right}", self._info(offset)

    def _info(self, offset):
        d = {"state": "active", "paused": self.paused,
             "owner": self.profile is not None, "score": round(self.last_score, 2),
             "smooth_cx": round(self.scx, 1), "smooth_h": round(self.sh, 1),
             "offset": None if offset is None else round(offset, 1), "lost": self.lost,
             "yaw_rate": round(self._yaw_rate(), 2), "w_des": round(self._w_des, 1)}
        if TURN_ENABLED:    # 黑匣子: 转向通道的原始量(只记不控, 供实机取阈值)
            d.update({"phase": self.phase,
                      "theta": None if self._theta is None else round(self._theta, 1),
                      "theta_raw": None if self._theta_raw is None else round(self._theta_raw, 1),
                      "theta_dot": round(self._theta_dot, 1),
                      "rate_world": round(self._rate_world(), 1),
                      "kf_q": round(self._kf_q, 2),
                      "exit_sign": self._exit_sign})
        return d


def run_follow_loop(controller):
    while controller.running:
        controller.tick()
        time.sleep(CYCLE_S)
