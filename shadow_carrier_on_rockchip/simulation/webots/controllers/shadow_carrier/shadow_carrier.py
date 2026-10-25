#!/usr/bin/env python3
"""Webots adapter for the current C3 ASCII protocol and RK-side HRI logic."""
import json
import math
import os
import random
import re
import sys
import tempfile
import time
from pathlib import Path

from controller import Supervisor


PROJECT_DIR = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(PROJECT_DIR / "scripts"))
sys.path.insert(0, str(PROJECT_DIR / "world" / "fusion"))
import hri_state
import follow_controller as _fc_mod
from follow_controller import FollowController
# 仿真轮径小(0.047m), BASE_SPD=100 只有 0.18m/s 跟不上步速; 仅仿真进程内调高, 实机不动
_fc_mod.BASE_SPD = 240
try:
    import sector_score   # [世界] HIDE 亲和度纯函数, 仿真 grid 快照直接复用真实现
except ImportError:
    sector_score = None


TIMEOUT_S = 0.45
RAMP_INTERVAL_S = 0.02
RAMP_STEP = 8
MAX_WHEEL_RAD_S = 10.0
SONAR_STOP_CM = 20.0
HRI_PERIOD_S = 0.30
WHEEL_RADIUS_M = 0.047      # 与世界 SIM_ROBOT 车轮几何一致(运动学积分用)
WHEEL_HALF_TRACK_M = 0.115
# --- V2 虚拟激光雷达 (SIM_LIDAR=1) ---
# 几何真值 360° 扫描 → 与真机【同一条】 lidar_guard 代码路径 → 命令过滤(限速/停车)。
# 仿真里没有两层板夹层结构 → 无铜柱, 遮罩为空; 位置/尺寸/速度标定按仿真底盘覆盖。
LIDAR_ON = os.environ.get("SIM_LIDAR") == "1"
LIDAR_PERIOD_S = 0.1        # 10Hz 一圈(与 X3 Pro 实机同档)
LIDAR_SCAN_H = 0.135        # V2 扫描平面高度(m, 两层板夹层)
LIDAR_STEP_DEG = 2.0        # 180 条射线
LIDAR_RANGE_MAX = 8.0       # 实用量程(与 lidar_guard.range_max_m 一致)
LIDAR_NOISE_M = 0.008       # 测距噪声 σ(复刻真实雷达抖动, 顺带压测去噪/去重)
# 走廊 demo 场景常量 (世界坐标 x, z)
DORM_DOOR_POS = (-0.85, 3.0)        # 宿舍门内侧(路人入场点)
HIDE_SPOT = (1.0, -4.3)             # 饮水机旁墙根: 离门 ~9m, 不挡饮水机/冰箱正前方
SPOT_B = (-1.2, -4.14)              # 备用躲避点(左墙, bearing≈-30°→扇区7, 无门/无家电): hide_occupied 回退目标
FOLLOW_KEEP_MIN_M = 2.6             # 仿真车距保持: 近于此距离刹车暂停跟随
FOLLOW_KEEP_RESUME_M = 3.0          # 远于此距离恢复推进(滞回)
SCENARIO = os.environ.get("SIM_SCENARIO", "baseline")   # baseline/cross2/occlude/bench_block/hide_occupied/handoff_cross/phantom/passby
# --- 云台冻结期的两项"仿真诚实度"开关 (2026-10-16) ---
# SIM_TURN=1: 打开底盘转向通道(原地转/掉头搜索); 实机由 follow_controller.TURN_ENABLED 控制
TURN_ON = os.environ.get("SIM_TURN") == "1"
# SIM_GT_TURN=0: 关掉 ground-truth 掉头假动作(_face_toward)。默认 1 = 保持历史剧本行为;
#   passby 场景必须置 0, 否则 WAIT 分支会用真值瞬移航向, 把"车自己转没转"验成假绿。
GT_TURN = os.environ.get("SIM_GT_TURN", "1") == "1"
# 水平 FOV 裁剪: 真机 640×480 相机水平半视场 ±32.2°(FX=508.2) → tan=0.63。
# 之前 _project_box 只按深度裁剪 → 主人在车侧 80° 也"看得见"(cx≈6000px),
# 让 follow 在仿真里永远不丢人; 真机 ±32° 外什么都没有 → 裁掉才与实机同构。
FOV_CULL_TAN = 0.63
FOV_LOG_INTERVAL_S = 1.0
PHANTOM_RATE = float(os.environ.get("SIM_PHANTOM_RATE",
                                    "0.35" if os.environ.get("SIM_SCENARIO") == "phantom" else "0"))
PHANTOM_SEED = int(os.environ.get("SIM_PHANTOM_SEED", "7"))

_fc_mod.TURN_ENABLED = TURN_ON  # 仿真内同步开关(实机不动)。2026-10-25 修: 旧代码只在
                                # TURN_ON 时设 True、从不设 False, 而 follow_controller 默认
                                # 已是 True(10-20 起) → SIM_TURN=0 的 passby off 臂失效成假对照


def clamp(value, low, high):
    return max(low, min(high, value))


class ShadowCarrierSimulation:
    def __init__(self):
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.self_node = self.robot.getSelf()
        self.position_field = self.self_node.getField("translation")
        self.rotation_field = self.self_node.getField("rotation")

        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.left_wheel_node = self.robot.getFromDef("LEFT_WHEEL")
        self.right_wheel_node = self.robot.getFromDef("RIGHT_WHEEL")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.pan_motor = self.robot.getDevice("pan motor")
        self.tilt_motor = self.robot.getDevice("tilt motor")
        self.pan_motor.setPosition(0.0)
        self.tilt_motor.setPosition(0.0)

        self.camera = self.robot.getDevice("front camera")
        self.camera.enable(self.timestep)
        self.camera.recognitionEnable(self.timestep)
        self.fx_px = (self.camera.getWidth() / 2.0) / math.tan(self.camera.getFov() / 2.0)
        self.sonar = self.robot.getDevice("front sonar")
        self.sonar.enable(self.timestep)
        self.keyboard = self.robot.getKeyboard()
        self.keyboard.enable(self.timestep)

        self.owner_node = self.robot.getFromDef("OWNER")
        self.owner_translation = self.owner_node.getField("translation")
        self.owner_start = list(self.owner_translation.getSFVec3f())
        self.passerby_node = self.robot.getFromDef("PASSERBY")
        self.passerby_translation = self.passerby_node.getField("translation")
        self.passerby_visible = False
        self.bottle_node = self.robot.getFromDef("OFFER_OBJECT")
        self.bottle_translation = self.bottle_node.getField("translation")
        self.bottle_hidden = list(self.bottle_translation.getSFVec3f())
        self.bottle_visible = False
        self.owner_visible = True

        self.left_target = 0.0
        self.right_target = 0.0
        self.left_current = 0.0
        self.right_current = 0.0
        self.command_deadline = 0.0
        self.last_ramp = time.monotonic()
        self.last_ascii = "STOP"
        self.manual_until = 0.0
        self.hri_enabled = True
        self.hri_action = "NONE"
        self.safe_target = None
        self.safe_target_yaw = None
        self.last_hri_tick = 0.0
        self.last_status = 0.0
        self.last_blocked = False

        self.autotest = bool(os.environ.get("SIM_AUTOTEST"))
        self.phantoms = []   # 幻影 person: [{x, z, life, conf}] — 检测层噪声, 非真实节点
        if PHANTOM_RATE > 0:
            random.seed(PHANTOM_SEED)
            print(f"[SIM] 幻影注入 ON: rate={PHANTOM_RATE} seed={PHANTOM_SEED}")
        self.autotest_motion = os.environ.get("SIM_AUTOTEST_MOTION") == "1"
        # 自动测试硬开几何真值检测(不依赖环境变量传递): 仿真不做 YOLO 精度, 只验行为
        self.geom_det = True if self.autotest else (os.environ.get("SIM_GEOM_DET") == "1")
        self.PERSON_W, self.PERSON_H = 0.62, 1.66
        self._geom_warned = False
        self.grid_path = Path(tempfile.gettempdir()) / "shadow_carrier_webots_grid.json"
        # A1.2 跨进程主人锁: 仿真 follow 侧发布 owner.json(格式与实机一致), HRI 侧消费
        # macOS/Windows 无 /dev/shm, 落在临时目录; 消费代码路径与实机完全相同
        self.owner_json_path = Path(tempfile.gettempdir()) / "shadow_carrier_webots_owner.json"
        self._last_owner_pub = 0.0
        self._last_ownerbox = None
        self._hold_far = False
        self.autopilot = False
        self.autopilot_t0 = 0.0
        self._kin_pos = [0.0, 0.06, 4.5]   # 与世界 SIM_ROBOT 出生点位姿一致
        self._kin_yaw = 0.49
        self._hide_driving = False
        self._hide_arrived = False
        self._drive_to_safe_target_done = False
        # V2 虚拟雷达: 与真机同一套 lidar_guard(仿真侧只替换扫描来源与标定)
        self.lidar_on = LIDAR_ON
        self.lidar_mod = None
        self.lidar_guard = None
        self.lidar_cfg = None
        self.lidar_env = None
        self.lidar_last = 0.0
        self.lidar_fid = 0
        self.lidar_rng = random.Random(11)
        self._lidar_last_log = 0.0
        self._sweep_bench_placed = False
        if self.lidar_on:
            try:
                import lidar_guard as _lg
                self.lidar_mod = _lg
                self.lidar_cfg = _lg.load_config(overrides={
                    "v_at_spd_max": MAX_WHEEL_RAD_S * WHEEL_RADIUS_M,
                    "v_nom_mps": 0.45, "v_min_mps": 0.12,
                    "pillars_xy": [], "mask_deg": [],
                    "range_max_m": 8.0,
                })
                self.lidar_guard = _lg.Guard(self.lidar_cfg)
                print(f"[SIM] 虚拟雷达 ON: 360° / {LIDAR_PERIOD_S * 1000:.0f}ms 一圈, "
                      f"扫描面 z={LIDAR_SCAN_H * 100:.1f}cm, "
                      f"v_at_spd_max={MAX_WHEEL_RAD_S * WHEEL_RADIUS_M:.2f}m/s")
            except Exception as error:
                self.lidar_on = False
                print(f"[SIM] 虚拟雷达不可用({error}), 退回无雷达模式")
        self._log_open()
        self.machine = self._new_state_machine()
        self.follower = FollowController(
            self._follow_cmd,
            person_provider=self._follow_person,
            yaw_deg_fn=self._yaw_deg,        # 转向通道: 死推算/到位判据(等价实机 IMU yaw_deg)
            yaw_rate_fn=self._yaw_rate_deg,  # 转向通道: 归因项(等价实机 IMU yaw_rate_dps)
            bearing_of_fn=self._bearing_of_cx,   # 用仿真自身内参解码(见该函数注释)
        )
        self.follower.start()
        print("[SIM] Webots adapter ready. HRI is on; use T to toggle it.")
        print("[SIM] 走廊 demo 场景: 宿舍门→饮水机→隔壁冰箱. WASD drive, Space stop;")
        print("[SIM] I/K/J/L move owner; G 主人自动驾驶(demo剧本); arrows move passerby, P toggle;")
        print("[SIM] O offer object; H hide owner; R reset. owner.json (A1.2 lock) is auto-published.")
        print(f"[SIM] sector_score {'loaded' if sector_score is not None else 'NOT FOUND (grid falls back to door-first)'}")
        print("[SIM] ',' '.' pan; '-' '=' tilt. Commands follow the C3 450 ms timeout and ramp.")
        print(f"[SIM] geom_det={self.geom_det} fx={self.fx_px:.1f}")
        if self.autotest:
            print("[SIM] AUTOTEST mode: scripted owner timeline, keyboard ignored, auto-quit at end.")
            print(f"[SIM] chassis motion: {'ON' if self.autotest_motion else 'OFF (observe-only)'}")

    def _set_person(self, x, z):
        self.owner_node.getField("translation").setSFVec3f([x, 0, z])

    def _set_passerby(self, position):
        """None=藏到视野外, 否则 [x, z] 落位"""
        self.passerby_visible = position is not None
        if position is None:
            self.passerby_translation.setSFVec3f([100, 0, 100])
        else:
            self.passerby_translation.setSFVec3f([position[0], 0, position[1]])

    def _set_object(self, offered, owner_pos):
        """手持物跟随主人手部; owner_pos = (x, z)"""
        if offered:
            self.bottle_node.getField("translation").setSFVec3f(
                [owner_pos[0] + 0.28, 0.95, owner_pos[1] - 0.05])
        else:
            self.bottle_node.getField("translation").setSFVec3f([100, 0.9, 100])

    @staticmethod
    def _lerp2(a, b, s):
        return (a[0] + (b[0] - a[0]) * s, a[1] + (b[1] - a[1]) * s)

    def _face_toward(self, target):
        """掉头面向目标点(排练转场用): 设内部航向, 积分器每步写出"""
        pos = self._kin_position()
        self._kin_yaw = math.atan2(-(target[0] - pos[0]), -(target[1] - pos[2]))

    def _scenario_post(self, t):
        """鲁棒性场景钩子: 在基线剧本(_demo_update)之后调用, 覆盖路人/主人/道具位置。
        每个场景只改自己关心的变量, 其余沿用基线。"""
        if SCENARIO == "cross2":
            # 基线只有一次路人交叉(P4); 这里在主人打水静止段(P2, 车在跟随推进)再加一次穿越,
            # 路人正好从"车-主人"连线中间穿过 → 配合几何遮挡, 真正考验 owner lock
            if 20.5 <= t < 26.5:
                s = (t - 20.5) / 6.0
                self._set_passerby(self._lerp2((-1.3, 2.2), (1.3, 1.6), s))
        elif SCENARIO == "occlude":
            # 主人在 P2 期间躲到冰箱后(左墙侧, 相对车被冰箱挡死) 8s → owner.json 过期
            # → lost 流程 → 回到 (0,-5) 重新可见。考验丢失-重捕获全链路
            if 22.0 <= t < 30.0:
                self.owner_translation.setSFVec3f([-1.25, 0, -4.75])
        elif SCENARIO == "bench_block":
            # 长椅搬进行车道中央(基线它在左墙边): 车跟随经过 z≈0.2 时必然 sonar BLOCK
            # → OBSTACLE STOP → t=28 撤走 → 恢复跟随。考验"受阻-等待-恢复"
            # 落位技巧: t=12 首次触发时把长椅放在**机器人当前车道**(x=robot.x)——
            # 固定放 x=0 会被机器人从旁边 ~1m 的空隙绕过(跟随贴主人车道, sonar 锥 ±10° 扫不到)
            bench = self.robot.getFromDef("BENCH")
            if bench is not None:
                f = bench.getField("translation")
                if 12.0 <= t < 28.0:
                    if not getattr(self, "_bench_placed", False):
                        rp = self._kin_position()
                        f.setSFVec3f([clamp(rp[0], -0.9, 0.9), 0.25, 0.2])
                        self._bench_placed = True
                        self._log({"type": "bench_placed",
                                   "at": [round(f.getSFVec3f()[0], 2), 0.2]})
                else:
                    f.setSFVec3f([-1.05, 0.25, 0.2])
                    self._bench_placed = False
        elif SCENARIO == "hide_occupied":
            # 路人从 t=20 起一直站在首选躲避点 HIDE_SPOT → 亲和度打分应把它降权/硬剔除,
            # HIDE 应改去备用点 SPOT_B(或门回退)。考验 sector_score 回退链
            if t >= 20.0:
                self._set_passerby(HIDE_SPOT)
        elif SCENARIO == "handoff_cross":
            # 交接窗口(P7-P8)路人垂直穿过"主人-车"视线 → owner.json 过期 2-3s,
            # 恰好踩在 owner_lost_abort_s=2.0 的门槛上。考验交接中锁主中断的鲁棒性
            if 63.0 <= t < 68.5:
                s = (t - 63.0) / 5.5
                self._set_passerby(self._lerp2((0.55, -3.4), (0.55, -4.9), s))
        elif SCENARIO == "passby":
            # 主人从车正前走到车正后(最近 1.0m 擦身, 0.8m/s): 云台冻结 + 水平 FOV ±32° 下,
            # 只有底盘自转能保住视野 —— 这是"原地转"通道的验收场景。
            # T0 起锁定"车-世界"参考系, 主人按直线走(不跟着车转), 与真实人的行为一致。
            T0, T1 = 6.0, 12.0
            if T0 <= t <= T1:
                ref = getattr(self, "_passby_ref", None)
                if ref is None:
                    rp = self._kin_position()
                    self._passby_ref = ref = (rp[0], rp[2], self._yaw())
                    self._log({"type": "passby_start",
                               "robot": [round(rp[0], 2), round(rp[2], 2)],
                               "yaw_deg": round(self._yaw_deg(), 1),
                               "turn_on": TURN_ON, "gt_turn": GT_TURN})
                s = (t - T0) / (T1 - T0)
                f = 2.6 - 4.8 * s                        # 前方 2.6m → 后方 2.2m
                lat = 0.35 + 0.65 * min(1.0, s * 3.0)    # 侧向 0.35 → 1.0m(避免起步横向跳变)
                fx, fz = -math.sin(ref[2]), -math.cos(ref[2])   # 车头前向(世界系)
                rx, rz = math.cos(ref[2]), -math.sin(ref[2])    # 车体右侧(世界系)
                self._set_person(ref[0] + fx * f + rx * lat,
                                 ref[1] + fz * f + rz * lat)
            elif t > T1 and getattr(self, "_passby_ref", None) is not None:
                self._passby_ref = None
                self._log({"type": "passby_end",
                           "owner": [round(v, 2)
                                     for v in self.owner_translation.getSFVec3f()[:2]]})
        elif SCENARIO == "lidar_sweep_block":
            # V2 雷达 360° 互补验证: 路人站在车【正后方 0.55m】——正面超声(±10°锥)完全看不见,
            # 但原地转向会把车尾扫向该路人 → 雷达的旋转净空应否决转向。
            # 车按"搜索主人/取景"的语义持续请求原地左转(实机 = hri_state 的转身动作)。
            # 判据: 超声全程 CLEAR + 雷达 rot 受阻 + veto 期间车没转起来。
            if 6.0 <= t < 26.0:
                self._place_passerby_behind()
                # 主人固定在饮水机旁(基线 P2 的位置) —— 否则剧本里的主人会从"停着的车"身上
                # 走过去, 触发的是超声(真人真在场), 会把"超声全程 CLEAR"的对照搞脏。
                self._set_person(0.0, -5.0)
                self._manual_command("MOVE L 70")
        elif SCENARIO == "phantom":
            # 幻影注入由 PHANTOM_RATE 全局生效, 剧本沿用基线 — 场景本身不改时间线
            pass

    def _place_passerby_behind(self):
        """lidar_sweep_block 用: 路人站在车正后方 0.40m(按车实时位姿算)。
        位置固定一次, 之后每拍重设(基线剧本每拍会把路人设成 None, 故必须每拍压回)。
        几何核算(离线夹具 + 实跑校准): 最近点 ≈0.21m < 车体扫掠圈 0.227 → 判"禁转"。
        0.45m 时最近点 0.315m → 只落到"转向限速"(rot_gap 0.16 → 仍给 64dps), 不构成否决。"""
        if not self._sweep_bench_placed:
            rp = self._kin_position()
            yaw = self._yaw()
            self._passerby_spot = (clamp(rp[0] + 0.40 * math.sin(yaw), -1.2, 1.2),
                                   clamp(rp[2] + 0.40 * math.cos(yaw), -5.8, 5.8))
            self._sweep_bench_placed = True
            self._log({"type": "passerby_placed", "mode": "behind_robot",
                       "at": [round(self._passerby_spot[0], 2),
                              round(self._passerby_spot[1], 2)]})
        self._set_passerby(self._passerby_spot)

    def _demo_update(self, t):
        """demo 全场景剧本(仿真秒): 跟出宿舍门→打水躲避→路人进门→隔壁取物回身递给车→一起回宿舍.
        返回 True 结束."""
        owner = tuple(self.owner_translation.getSFVec3f()[:2])
        if t < 18:            # P1 跟出宿舍门走到饮水机 (8.05m @0.45m/s), 车保持 ~2.5m 车距
            self._set_person(*self._lerp2((-0.8, 3.0), (0.0, -5.0), t / 18))
            self._set_object(False, (0, 0)); self._set_passerby(None)
        elif t < 40:          # P2 主人打水静止: 5s→WAIT, 20s→HIDE(38s)→车挪到饮水机旁墙根
            self._set_person(0.0, -5.0); self._set_object(False, (0, 0))
            self._set_passerby(None)
        elif t < 42:          # P3 路人出现在宿舍门口
            self._set_person(0.0, -5.0); self._set_object(False, (0, 0))
            self._set_passerby(DORM_DOOR_POS)
        elif t < 54:          # P4 路人沿走廊走向饮水机(经过车前, 验证锁主)
            self._set_person(0.0, -5.0); self._set_object(False, (0, 0))
            self._set_passerby(self._lerp2(DORM_DOOR_POS, (0.3, -5.2), (t - 42) / 12))
        elif t < 58:          # P5 主人到旁边冰箱取物 (1.03m @0.26m/s, 始终在车视野内)
            self._set_person(*self._lerp2((0.0, -5.0), (-0.55, -4.6), (t - 54) / 4))
            self._set_object(False, (0, 0)); self._set_passerby((0.3, -5.2))
        elif t < 62:          # P6 取物, 60s 拿起东西
            self._set_person(-0.55, -4.6)
            self._set_object(t >= 60, (-0.55, -4.6)); self._set_passerby((0.3, -5.2))
        elif t < 67:          # P7 持物缓近递给车 (0.95m @0.19m/s → RECEIVE, 车随后上前)
            pos = self._lerp2((-0.55, -4.6), (0.1, -4.1), (t - 62) / 5)
            self._set_person(*pos); self._set_object(True, pos)
            self._set_passerby((0.3, -5.2))
        elif t < 71:          # P8 递给它(停住), 车上前一点接住
            self._set_person(0.1, -4.1); self._set_object(True, (0.1, -4.1))
            self._set_passerby((0.3, -5.2))
        elif t < 73.5:        # P9 交接完成(轻微后退), 放下东西
            pos = self._lerp2((0.1, -4.1), (0.0, -3.5), (t - 71) / 2.5)
            self._set_person(*pos); self._set_object(False, pos)
            self._set_passerby((0.3, -5.2))
        elif t < 74:          # P10 侧身转向宿舍(唤醒恢复跟随)
            self._set_person(0.85, -3.3); self._set_object(False, (0, 0))
            self._set_passerby((0.3, -5.2))
        elif t < 86:          # P11 一起回宿舍: 主人走前面, 车保持车距跟随
            self._set_person(*self._lerp2((0.85, -3.3), (0.0, 2.6), (t - 74) / 12))
            self._set_object(False, (0, 0)); self._set_passerby((0.3, -5.2))
        elif t < 91:          # P12 到宿舍门口静止 → WAIT, 车在身后等候
            self._set_person(0.0, 2.6); self._set_object(False, (0, 0))
            self._set_passerby((0.3, -5.2))
        else:
            return True
        return False


    def _new_state_machine(self):
        params = hri_state.load_params()
        params["paths"]["grid_json"] = str(self.grid_path)
        # 用仿真相机真实焦距覆盖标定值, 让距离/速度阈值在仿真里也成立
        params["camera"]["focal_px"] = (self.camera.getWidth() / 2.0) / math.tan(
            self.camera.getFov() / 2.0)
        # 仿真人形肩宽(含手臂)≈0.62m, 与真机假设 0.45m 不同, 覆盖以让距离估计成立
        params["camera"]["shoulder_width_m"] = 0.62
        # A1.2: 打开跨进程主人锁消费端(实机由 hri_params.json 的 owner.source 控制拨盘)
        params["owner"]["source"] = "file"
        params["owner"]["file"] = str(self.owner_json_path)
        return hri_state.HRIStateMachine(
            send_cmd_fn=self._on_hri_action,
            log_fn=self._hri_log,
            params=params,
        )

    def _log_open(self):
        # SIM_LOG: 给并行/自动化跑用(多人同时跑同一 world 时默认路径会互相覆盖成垃圾)
        path = Path(os.environ.get("SIM_LOG")
                    or (Path(__file__).resolve().parent / "sim_session.jsonl"))
        self.log_file = open(path, "w", buffering=1, encoding="utf-8")  # 每次运行覆盖, 避免多会话混在一起
        self.log_t0 = time.monotonic()
        self.log_file.write(json.dumps({"type": "session_start",
                                        "t0": time.time(),
                                        "scenario": SCENARIO,
                                        "geom_det": self.geom_det,
                                        "autotest": self.autotest,
                                        "lidar": bool(self.lidar_on),
                                        "turn_on": TURN_ON,
                                        "gt_turn": GT_TURN,
                                        "fov_cull_tan": FOV_CULL_TAN,
                                        "fx_px": round(self.fx_px, 1)}) + "\n")
        print(f"[SIM] 黑匣子: {path}")
        print(f"[SIM] scenario = {SCENARIO}")

    def _log(self, record):
        if not getattr(self, "log_file", None):
            return
        record["t"] = round(time.monotonic() - self.log_t0, 2)
        self.log_file.write(json.dumps(record, ensure_ascii=False) + "\n")

    def _hri_log(self, msg):
        print(msg)
        self._log({"type": "transition", "msg": msg})

    def _yaw(self):
        return self._kin_yaw

    def _yaw_deg(self):
        """IMU 等价量: 积分航向(度)。仿真 yaw 正 = 左转 = 逆时针, 与实机 IMU 约定一致"""
        return math.degrees(self._kin_yaw)

    def _yaw_rate_deg(self):
        """IMU 等价量: 偏航角速度(度/秒, 逆时针为正)。用与积分器同一套轮速→ω 换算,
        这样 follow 的归因项(θ̇ + IMU_TO_THETA_SIGN·yaw_rate)在仿真里也成立。"""
        scale = MAX_WHEEL_RAD_S / 255.0
        omega = (self.right_current - self.left_current) / 2.0 * scale \
            * WHEEL_RADIUS_M / WHEEL_HALF_TRACK_M
        return math.degrees(omega)

    def _bearing_of_cx(self, cx):
        """像素列 → 车体系方位角, 用**仿真投影自己的内参**(fx_px/CX=W/2)。
        实机 follow 默认走 state/decoder(真机内参 508.2/292.7); 若仿真也用它,
        两套内参不一致会引入常数偏置 + 尺度误差(实测 +3.1° / ×1.15) → 转向通道
        的 θ 是错的。所以这里按仿真的真实模型解码, 与 _project_box 严格互逆。"""
        return math.degrees(math.atan((cx - self.camera.getWidth() / 2.0) / self.fx_px))

    def _kin_position(self):
        """机器人运动学位姿(自持状态, 不读 Webots —— 无 physics 节点读数不可靠)"""
        return self._kin_pos

    def _bearing_to(self, target):
        position = self._kin_position()
        dx, dz = target[0] - position[0], target[2] - position[2]
        yaw = self._yaw()
        local_x = math.cos(yaw) * dx - math.sin(yaw) * dz
        local_z = math.sin(yaw) * dx + math.cos(yaw) * dz
        return math.degrees(math.atan2(local_x, -local_z))

    def _grid_object(self, def_name, cls):
        node = self.robot.getFromDef(def_name)
        if node is None:
            return None
        pos = node.getPosition()
        robot_pos = self._kin_position()
        dist = math.hypot(pos[0] - robot_pos[0], pos[2] - robot_pos[2])
        return {"cls": cls, "bearing_deg": self._bearing_to(pos),
                "dist_m": round(dist, 2)}

    def _refresh_grid(self):
        """走廊 demo 场景的语义快照: 宿舍门/隔壁门 + 饮水机 + 冰箱 + sector_scores"""
        objects = [o for o in (
            self._grid_object("DORM_DOOR", "door"),
            self._grid_object("SIDE_DOOR", "door"),
            self._grid_object("WATER_DISPENSER", "water dispenser"),
            self._grid_object("REFRIGERATOR", "refrigerator"),
            self._grid_object("PASSERBY", "person") if self.passerby_visible else None,
        ) if o]
        # 幻影 person 也进网格(镜像实机: YOLO 检测喂 grid) — 与真 person 同权重(当前无权重)
        rp_grid = self._kin_position()
        for ph in self.phantoms:
            objects.append({"cls": "person",
                            "bearing_deg": self._bearing_to((ph["x"], 0.0, ph["z"])),
                            "dist_m": round(math.hypot(ph["x"] - rp_grid[0],
                                                       ph["z"] - rp_grid[2]), 2)})
        # 自由方向 = 已知躲避点(主/备)相对车的方位角(仿真实机滚动地图的自由扇区)
        # SPOT_B 仅 hide_occupied 场景注入(模拟"机器人知道两个候选躲避点");
        # 基线只认一个点, 避免 margin 规则改变已验证的基线选点
        free = []
        rp = self._kin_position()
        yaw = self._yaw()
        spots = (HIDE_SPOT, SPOT_B) if SCENARIO == "hide_occupied" else (HIDE_SPOT,)
        for spot in spots:
            dx, dz = spot[0] - rp[0], spot[1] - rp[2]
            lx = math.cos(yaw) * dx - math.sin(yaw) * dz
            lz = math.sin(yaw) * dx + math.cos(yaw) * dz
            free.append(round(math.degrees(math.atan2(lx, -lz)), 1))
        snapshot = {
            "ts": time.time(),
            "objects": objects,
            "free_directions_deg": free,
        }
        if sector_score is not None:
            # 与实机 sector_score.py 同一 schema: 产出 sector_scores 供 HIDE pick_safe_spot 消费
            snapshot["sector_scores"] = sector_score.score_map({
                "doors": [{"bearing_deg": o["bearing_deg"], "age_s": 0}
                          for o in objects if "door" in o["cls"]],
                "free_sectors": [{"bearing_deg": b, "age_s": 0}
                                 for b in free],
                "objects": [{"cls": o["cls"], "bearing_deg": o["bearing_deg"], "age_s": 0}
                            for o in objects],
            })
        try:
            self.grid_path.write_text(json.dumps(snapshot), encoding="utf-8")
        except OSError as error:
            print(f"[SIM] grid snapshot unavailable: {error}")

    def _publish_owner_json(self, bbox):
        """仿真 follow 侧的主人锁发布, 镜像实机 follow_controller._publish_owner:
        {ts, bbox} 原子替换, 限频 0.2s; 主人不可见时不写 → 文件自然过期, 消费方回退"""
        now = time.time()
        if now - self._last_owner_pub < 0.2:
            return
        self._last_owner_pub = now
        self._last_ownerbox = [float(v) for v in bbox]
        try:
            tmp = self.owner_json_path.with_suffix(".tmp")
            tmp.write_text(json.dumps({"ts": round(now, 3),
                                       "bbox": [round(float(v), 1) for v in bbox]}),
                           encoding="utf-8")
            os.replace(tmp, self.owner_json_path)
        except OSError as error:
            print(f"[SIM] owner.json publish error: {error}")

    def _project_box(self, x, z, w_m, h_m):
        """世界 (x,z) 的 Person 尺寸 → 针孔投影 bbox; 深度窗外/水平出视场返回 None"""
        rp = self._kin_position()
        yaw = self._yaw()
        dx, dz = x - rp[0], z - rp[2]
        lx = math.cos(yaw) * dx - math.sin(yaw) * dz
        lz = math.sin(yaw) * dx + math.cos(yaw) * dz
        depth = -lz   # 车前方为 -z
        if depth < 0.15 or depth > 6.0:
            return None
        # 水平视场裁剪(云台冻结 = 相机与车体固连): 出画即无检测, 与真机一致
        lateral = lx / depth
        if abs(lateral) > FOV_CULL_TAN:
            now = self.robot.getTime()
            if now - getattr(self, "_fov_log_t", -9.0) >= FOV_LOG_INTERVAL_S:
                self._fov_log_t = now
                self._log({"type": "fov_exit",
                           "side": "R" if lateral > 0 else "L",
                           "bearing_deg": round(math.degrees(math.atan(lateral)), 1),
                           "dist_m": round(math.hypot(dx, dz), 2)})
            return None
        bw = self.fx_px * w_m / depth
        bh = self.fx_px * h_m / depth
        cx = self.camera.getWidth() / 2 + self.fx_px * lateral
        cy = self.camera.getHeight() / 2
        return [cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2]

    def _synthetic_box(self, node, w_m, h_m):
        """3D 位置 + 针孔模型合成检测框; 用车体真实位姿投影(运动模式必须)。"""
        pos = node.getField("translation").getSFVec3f()
        return self._project_box(pos[0], pos[2], w_m, h_m)

    PROP_H = {"BENCH": 0.5, "WATER_DISPENSER": 1.1, "REFRIGERATOR": 1.7}

    def _prop_boxes(self):
        """道具 AABB (x, z, half_x, half_z, 高度m), 读实时位置(bench_block 会搬动长椅)。
        高度用于感知遮挡: 0.5m 长椅挡声呐(9.5cm 高)但不挡站立者的视线。"""
        boxes = []
        for def_name, half in (("BENCH", (0.175, 0.6)),
                               ("WATER_DISPENSER", (0.225, 0.21)),
                               ("REFRIGERATOR", (0.3, 0.3))):
            node = self.robot.getFromDef(def_name)
            if node is not None:
                p = node.getPosition()
                boxes.append((p[0], p[2], half[0], half[1], self.PROP_H[def_name]))
        return boxes

    def _person_boxes(self, exclude=None):
        boxes = []
        if self.passerby_visible:
            p = self.passerby_translation.getSFVec3f()
            boxes.append((p[0], p[2], 0.31, 0.19, "PASSERBY", 1.66))
        if exclude != "OWNER":
            o = self.owner_translation.getSFVec3f()
            boxes.append((o[0], o[2], 0.31, 0.19, "OWNER", 1.66))
        return boxes

    def _seg_blocked(self, ax, az, bx, bz, exclude, target_h):
        """2D 线段-AABB 相交 + 高度感知: 道具仅在 prop_h >= target_h - 0.35 时遮挡视线
        (0.5m 长椅挡不住 1.66m 人的视线; 人与人间永远互相遮挡)。"""
        occluders = [(x, z, hx, hz, name, h) for (x, z, hx, hz, name, h)
                     in self._person_boxes(exclude)]
        occluders += [(x, z, hx, hz, "", h) for (x, z, hx, hz, h) in self._prop_boxes()]
        for ox, oz, hx, hz, name, h in occluders:
            if name == exclude:
                continue
            if name == "" and h < target_h - 0.35:
                continue   # 矮道具不挡更高目标的视线
            dx, dz = bx - ax, bz - az
            tmin, tmax = 0.0, 1.0
            for a, b, lo, hi in ((ax, dx, ox - hx, ox + hx), (az, dz, oz - hz, oz + hz)):
                if abs(b) < 1e-9:
                    if a < lo or a > hi:
                        tmin, tmax = 1.0, 0.0   # 平行且在外 → 无交
                        break
                else:
                    t1, t2 = (lo - a) / b, (hi - a) / b
                    if t1 > t2:
                        t1, t2 = t2, t1
                    tmin, tmax = max(tmin, t1), min(tmax, t2)
                    if tmin > tmax:
                        break
            if tmin <= tmax and tmax >= 0.0 and tmin <= 1.0:
                return True
        return False

    @staticmethod
    def _ray_dist_m(ax, az, dx, dz, ox, oz, hx, hz, seg_len):
        """射线 (ax,az) 方向 (dx,dz, 须为单位向量) 与 AABB 的入射距离【米】;
        超段长 seg_len 或无交 → None。
        ⚠️ 2026-10-16 修正: 原 `_ray_aabb_t` 返回的 t 已是米(单位方向量的参数), 但两个调用方
        又乘了段长 → 距离被放大(声呐 2×, 且 1m 之外的命中被 tmax≤1 判成"无交")。
        真机声呐 20cm 阈值下这个偏差让它等价于 10cm 才停 —— 仿真比实机更宽松, 属安全相关错误。"""
        tmin, tmax = 0.0, float(seg_len)
        for a, b, lo, hi in ((ax, dx, ox - hx, ox + hx), (az, dz, oz - hz, oz + hz)):
            if abs(b) < 1e-9:
                if a < lo or a > hi:
                    return None
            else:
                t1, t2 = (lo - a) / b, (hi - a) / b
                if t1 > t2:
                    t1, t2 = t2, t1
                tmin, tmax = max(tmin, t1), min(tmax, t2)
                if tmin > tmax:
                    return None
        return tmin

    def _virtual_sonar_cm(self):
        """虚拟超声波(cm): Webots sonar 设备在本世界不返回数据(自测: 直冲端墙 30s 恒 200,
        2026-10-11), 改用几何真值 5 射线(±10°, 对应设备 aperture 0.35)求最近障碍,
        喂给与真机相同的 blocked 行为代码路径。遮挡体 = 道具 + 四面墙 + 人。
        2026-10-16: 距离换算修正(见 _ray_dist_m), 现在 0.2–2.0m 内的读数与真机同标定。"""
        rp = self._kin_position()
        yaw = self._yaw()
        best_m = 2.0
        boxes = [b[:4] for b in self._prop_boxes()] + [b[:4] for b in self._person_boxes()]
        boxes += [(-1.4, 0.0, 0.05, 6.2), (1.4, 0.0, 0.05, 6.2),
                  (0.0, -6.05, 1.45, 0.05), (0.0, 6.05, 1.45, 0.05)]
        for da in (-0.175, -0.0875, 0.0, 0.0875, 0.175):
            a = yaw + da
            dx, dz = -math.sin(a), -math.cos(a)   # 前进方向, 与运动学积分一致
            d_best = None
            for (ox, oz, hx, hz) in boxes:
                d = self._ray_dist_m(rp[0], rp[2], dx, dz, ox, oz, hx, hz, 2.0)
                if d is not None and (d_best is None or d < d_best):
                    d_best = d
            best_m = min(best_m, d_best if d_best is not None else 2.0)
        return best_m * 100.0

    def _virtual_scan(self):
        """几何真值虚拟雷达帧: 360° × LIDAR_STEP_DEG 射线, 与道具/墙/人求交。
        高度感知: 只有"穿过扫描平面 z=LIDAR_SCAN_H"的物体才返回(桌面板/矮于平面的东西
        真机也看不见——仿真必须一样, 否则测不出真实的失效边界)。
        方位角约定与 lidar_guard / [世界] 同源: 0°=车头正前, 正=物理右。"""
        rp = self._kin_position()
        yaw = self._yaw()
        boxes = [(x, z, hx, hz) for (x, z, hx, hz, h) in self._prop_boxes()
                 if h >= LIDAR_SCAN_H]
        boxes += [(x, z, hx, hz) for (x, z, hx, hz, _n, h) in self._person_boxes()
                  if h >= LIDAR_SCAN_H]
        boxes += [(-1.4, 0.0, 0.05, 6.2), (1.4, 0.0, 0.05, 6.2),
                  (0.0, -6.05, 1.45, 0.05), (0.0, 6.05, 1.45, 0.05)]
        pts = []
        for b_deg in range(0, 360, int(LIDAR_STEP_DEG)):
            b = math.radians(b_deg)
            dx, dz = math.sin(b - yaw), -math.cos(b - yaw)
            d_best = None
            for (ox, oz, hx, hz) in boxes:
                d = self._ray_dist_m(rp[0], rp[2], dx, dz, ox, oz, hx, hz, LIDAR_RANGE_MAX)
                if d is not None and (d_best is None or d < d_best):
                    d_best = d
            if d_best is None:
                continue                       # 该方向无回波(超过量程)
            rng = d_best + self.lidar_rng.gauss(0.0, LIDAR_NOISE_M)
            if rng <= 0.05:
                continue
            pts.append([b_deg, round(rng, 3)])
        return {"frame_id": self.lidar_fid, "ts": time.monotonic(), "points": pts}

    def _lidar_update(self, now):
        """10Hz 更新虚拟雷达 → 安全包络(与真机 daemon 同一条代码路径)"""
        if not self.lidar_on or now - self.lidar_last < LIDAR_PERIOD_S:
            return
        self.lidar_last = now
        self.lidar_fid += 1
        self.lidar_env = self.lidar_guard.update(self._virtual_scan(), now=now)

    def _lidar_filter(self, command):
        """命令过滤: 与 rk_control 的 uart_send 同语义(受阻→STOP, 受限→限速)"""
        if not self.lidar_on or self.lidar_env is None:
            return command, None
        try:
            return self.lidar_mod.apply_to_ascii(command, self.lidar_env, self.lidar_cfg)
        except Exception:
            return command, None

    def _lidar_log_fields(self):
        env = self.lidar_env
        if not self.lidar_on or env is None:
            return {}
        blocked = {k: v for k, v in (env.get("blocked") or {}).items() if v}
        return {"lidar": {"mode": env.get("mode"), "n": env.get("n_points"),
                          "blocked": sorted(blocked),
                          "gap": {"fwd": env["gap_m"].get("fwd"),
                                  "back": env["gap_m"].get("back"),
                                  "rot": env.get("rot_gap_m")},
                          "vx_max": env["limits"].get("vx_max"),
                          "wz_max": env["limits"].get("wz_max_dps"),
                          "legs": len(env.get("person_legs") or []),
                          "escape": (env.get("escape") or {}).get("dir")}}

    def _phantoms_update(self):
        """幻影 person 检测注入(复刻实机夜景失败模式: 宿舍 95/98 帧含幻影):
        每拍以 PHANTOM_RATE 概率在车前随机位置生成一个存活 3-10 拍的假 person 框(成簇)。
        只进检测列表与网格快照, 不是真实节点 — 检验认主回退通道与 HIDE/RECEIVE 的抗噪。"""
        if PHANTOM_RATE <= 0:
            return
        if random.random() < PHANTOM_RATE:
            rp = self._kin_position()
            a = self._kin_yaw
            d = random.uniform(1.2, 4.5)
            px = clamp(rp[0] - math.sin(a) * d + random.uniform(-1.2, 1.2), -1.25, 1.25)
            pz = clamp(rp[2] - math.cos(a) * d + random.uniform(-1.0, 1.0), -5.8, 5.8)
            self.phantoms.append({"x": px, "z": pz,
                                  "life": random.randint(3, 10),
                                  "conf": round(random.uniform(0.55, 0.9), 2)})
        for ph in self.phantoms:
            ph["life"] -= 1
        self.phantoms[:] = [p for p in self.phantoms if p["life"] > 0]

    def _phantom_detections(self):
        out = []
        for ph in self.phantoms:
            bbox = self._project_box(ph["x"], ph["z"], self.PERSON_W * 0.8, self.PERSON_H)
            if bbox is not None:
                out.append({"label": "person", "conf": ph["conf"], "bbox": bbox})
        return out

    def _owner_lock_stale(self):
        """owner.json 是否过期(>1s 未发布) — 过期时 _pick_owner 会回退到 conf 最大 person,
        幻影由此进入'被当成主人'的通道(实机夜景病根的 HRI 侧残留)"""
        try:
            o = json.loads(self.owner_json_path.read_text(encoding="utf-8"))
            return time.time() - o.get("ts", 0) > 1.0
        except Exception:
            return True

    def _geom_detections(self):
        """几何真值检测: 主人 / 手持物 / 路人(可见时); 带物体遮挡判断(§鲁棒性:
        路人穿过主人-车连线 / 主人躲在冰箱后 → 检测框消失, 与实机 YOLO 行为对齐)"""
        out = []
        for label, node, w_m, h_m, exclude, target_h in (
                ("person", self.owner_node, self.PERSON_W, self.PERSON_H, "OWNER", 1.66),
                ("bottle", self.bottle_node, 0.09, 0.22, "OWNER", 0.95),
                ("person", self.passerby_node, self.PERSON_W, self.PERSON_H, "PASSERBY", 1.66)):
            if node is None:
                continue
            if node is self.passerby_node and not self.passerby_visible:
                continue
            bbox = self._synthetic_box(node, w_m, h_m)
            if bbox is None:
                continue
            pos = node.getField("translation").getSFVec3f()
            rp = self._kin_position()
            if self._seg_blocked(rp[0], rp[2], pos[0], pos[2], exclude, target_h):
                continue
            out.append({"label": label, "conf": 0.95, "bbox": bbox})
        return out + self._phantom_detections()

    def _detections(self):
        if self.geom_det:
            return self._geom_detections()
        supported = {
            "person", "bottle", "cup", "wine glass", "banana", "apple",
            "orange", "handbag", "backpack", "door",
        }
        detections = []
        for item in self.camera.getRecognitionObjects():
            label = (item.getModel() or "").strip().lower()
            if label not in supported:
                continue
            cx, cy = item.getPositionOnImage()
            width, height = item.getSizeOnImage()
            if width <= 0 or height <= 0:
                continue
            detections.append({
                "label": label,
                "conf": 0.95,
                "bbox": [cx - width / 2, cy - height / 2,
                         cx + width / 2, cy + height / 2],
            })
        return detections

    def _owner_lock_box(self):
        """仿真 follow 的认主结果: 恒等主人(模板匹配成功的化身), 与实机模板锁主对齐。
        双人场景里路人再大再近也不影响这里——主/路人区分只由 OWNER 节点定义。"""
        if self.geom_det:
            return self._synthetic_box(self.owner_node, self.PERSON_W, self.PERSON_H)
        return self._recognition_owner_box()

    def _publish_owner_lock(self):
        """镜像实机 follow 进程的发布行为: 模板锁住主人就写 owner.json (限频 0.2s)。
        独立于 follower.tick() —— autotest 观察模式不 tick 跟随, 但锁发布照常。"""
        bbox = self._owner_lock_box()
        if bbox is not None:
            self._publish_owner_json(bbox)

    def _follow_person(self):
        bbox = self._owner_lock_box()
        if bbox is None:
            return None   # 主人不可见: 不发布 → owner.json 过期, 消费方回退(与实机一致)
        x1, y1, x2, y2 = bbox
        return {
            "cx": (x1 + x2) / 2,
            "cy": (y1 + y2) / 2,
            "h": y2 - y1,
        }

    def _recognition_owner_box(self):
        """Webots 识别模式: 主人由红色 recognitionColors (0.82 0.20 0.16) 认出"""
        for item in self.camera.getRecognitionObjects():
            label = (item.getModel() or "").strip().lower()
            if label != "person":
                continue
            for color in item.getColors():
                if all(abs(c - t) < 0.05 for c, t in zip(color, (0.82, 0.20, 0.16))):
                    cx, cy = item.getPositionOnImage()
                    width, height = item.getSizeOnImage()
                    if width > 0 and height > 0:
                        return [cx - width / 2, cy - height / 2,
                                cx + width / 2, cy + height / 2]
                    return None
        return None

    def _set_targets(self, left_pwm, right_pwm):
        self.left_target = float(clamp(left_pwm, -255, 255))
        self.right_target = float(clamp(right_pwm, -255, 255))
        if self.left_target or self.right_target:
            self.command_deadline = time.monotonic() + TIMEOUT_S
        self.last_ascii = "DRIVE"

    def _execute_ascii(self, command):
        # V2 雷达避障收口: 与实机同位置(命令进入"固件"前的最后一道)。惰性: 雷达未开时原样。
        if self.lidar_on and self.lidar_env is not None:
            command, _note = self._lidar_filter(command)
            if _note and time.monotonic() - self._lidar_last_log >= 0.5:   # 节流: 黑匣子不刷屏
                self._lidar_last_log = time.monotonic()
                self._log({"type": "lidar_filter", "cmd": command, "note": _note})
                print(f"[SIM] lidar filter: {_note} → {command}")
        parts = command.strip().upper().split()
        if not parts:
            return
        action = parts[0]
        if action == "STOP" and len(parts) == 1:
            self.left_target = self.right_target = 0.0
            self.left_current = self.right_current = 0.0
            self.command_deadline = 0.0
            self.last_ascii = "STOP"
        elif action == "PING" and len(parts) == 1:
            print("[SIM] PONG")
        elif action == "MOVE" and len(parts) == 3 and parts[1] in {"F", "B", "L", "R"}:
            try:
                speed = int(parts[2])
            except ValueError:
                self._execute_ascii("STOP")
                return
            if not 0 <= speed <= 255:
                self._execute_ascii("STOP")
                return
            values = {
                "F": (speed, speed),
                "B": (-speed, -speed),
                "L": (-speed, speed),
                "R": (speed, -speed),
            }
            self._set_targets(*values[parts[1]])
            self.last_ascii = " ".join(parts)
        elif action == "DIFF" and len(parts) == 3:
            match_left = re.fullmatch(r"L(\d{1,3})", parts[1])
            match_right = re.fullmatch(r"R(\d{1,3})", parts[2])
            if not match_left or not match_right:
                self._execute_ascii("STOP")
                return
            left, right = int(match_left.group(1)), int(match_right.group(1))
            if left > 255 or right > 255:
                self._execute_ascii("STOP")
                return
            self._set_targets(left, right)
            self.last_ascii = f"DIFF L{left} R{right}"
        elif action in {"PAN", "TLT"} and len(parts) == 2:
            try:
                v = float(parts[1])
            except ValueError:
                return
            if not math.isfinite(v):    # "PAN nan": clamp 的 min/max 对 nan 比较序会落到 180
                return                  # (顶死限位), 非有限值直接拒收(2026-10-25 §7.1-④)
            angle = clamp(v, 0.0, 180.0)
            if action == "PAN":
                self.pan_motor.setPosition(math.radians(90.0 - angle))
            else:
                self.tilt_motor.setPosition(math.radians(angle - 112.0))
            self.last_ascii = f"{action} {angle:.1f}"
        else:
            self._execute_ascii("STOP")
            print(f"[SIM] rejected command: {command!r}")

    def _on_hri_action(self, action):
        if time.monotonic() < self.manual_until:
            return
        if self.autotest and not self.autotest_motion:
            # 观察模式: 只记录 HRI 意图, 不驱动底盘(否则恒速跟随会顶住主人)
            if action != self.hri_action:
                self.hri_action = action
                self._log({"type": "hri_action", "action": action})
            return
        if action == self.hri_action:
            if action == "BACK_OFF":
                self._execute_ascii("MOVE B 60")
            elif action == "APPROACH":
                self._execute_ascii("MOVE F 90")
            return   # GOTO_SAFE 不重复设目标: 目标固定才能在推力漂移下收敛
        self.hri_action = action
        if action == "NONE":
            self.safe_target = None
            self.safe_target_yaw = None
            self._hide_driving = False
            self._hide_arrived = False
            self._drive_to_safe_target_done = False
            self._execute_ascii("STOP")
        elif action == "BACK_OFF":
            self.safe_target = None
            self.safe_target_yaw = None
            self._hide_driving = False
            self._hide_arrived = False
            self._drive_to_safe_target_done = False
            self._execute_ascii("MOVE B 60")
        elif action == "APPROACH":
            self.safe_target = None
            self.safe_target_yaw = None
            self._hide_driving = False
            self._hide_arrived = False
            self._drive_to_safe_target_done = False
            self._execute_ascii("MOVE F 90")
        elif action.startswith("GOTO_SAFE"):
            self._set_safe_target(action)

    def _set_safe_target(self, action):
        """GOTO_SAFE → 目标点: 方位角命中已知躲避点(±55°)直接以该点为目标(航向取几何朝向);
        否则回退为沿方位角 1m 航点(旧行为)"""
        match = re.search(r"GOTO_SAFE\s+(-?\d+(?:\.\d+)?)", action)
        bearing = float(match.group(1)) if match else 0.0
        position = self._kin_position()
        for spot in (HIDE_SPOT, SPOT_B):
            diff = abs((self._bearing_to((spot[0], 0.0, spot[1])) - bearing + 180) % 360 - 180)
            if diff <= 55:
                # 航向直接由几何朝向给出(与 _face_toward 同一约定), 不经扇区角换算
                self.safe_target_yaw = math.atan2(-(spot[0] - position[0]),
                                                  -(spot[1] - position[2]))
                self.safe_target = [spot[0], position[1], spot[1]]
                return
        yaw_target = self._yaw() - math.radians(bearing)
        self.safe_target_yaw = yaw_target
        self.safe_target = [
            position[0] - math.sin(yaw_target),
            position[1],
            position[2] - math.cos(yaw_target),
        ]

    def _drive_to_safe_target(self):
        """纯追踪(arc)驱车到躲避点: 与跟随弧线同款控制, 不原地旋转
        (macOS 求解器下急旋会诱发接触弹射, 曾把车甩向主人)"""
        if self.safe_target is None:
            self._hide_driving = False
            return
        position = self._kin_position()
        remaining = math.hypot(self.safe_target[0] - position[0],
                               self.safe_target[2] - position[2])
        if remaining <= 0.5:
            self._hide_driving = False
            self._drive_to_safe_target_done = True
            self._execute_ascii("STOP")
            return
        self._hide_driving = True
        err = self._bearing_to((self.safe_target[0], 0.0, self.safe_target[2]))  # + = 右
        base, slow_min = 220, 70
        if remaining < 0.9:
            base = 140   # 近目标减速, 防止纯追踪极限环
        ratio = min(1.0, abs(err) / 50.0)
        slow = int(base - ratio * (base - slow_min))
        if err > 4:      # 目标在右 → 右轮慢(与 follow_controller 同款映射)
            cmd = f"DIFF L{base} R{slow}"
        elif err < -4:   # 目标在左 → 左轮慢
            cmd = f"DIFF L{slow} R{base}"
        else:
            cmd = f"DIFF L{base} R{base}"
        self._log({"type": "hide_drive", "err": round(err, 1),
                   "remaining": round(remaining, 2), "cmd": cmd})
        self._execute_ascii(cmd)

    def _owner_gt_dist(self):
        """与主人的地面真值距离(supervisor 才有, 用于车距保持)"""
        op = self.owner_translation.getSFVec3f()
        rp = self._kin_position()
        return math.hypot(op[0] - rp[0], op[2] - rp[2])

    def _follow_cmd(self, command):
        """仿真专用车距保持: 跟随推进命令(DIFF)在离主人过近时改发 STOP 刹车, 保持 ~2.5m 跟随距离。
        只过滤跟随器输出; APPROACH/BACK_OFF/GOTO(动作层)与丢主倒车(MOVE B)不受影响。"""
        if command.startswith("DIFF"):
            dist = self._owner_gt_dist()
            if dist is not None:
                if dist < FOLLOW_KEEP_MIN_M:
                    self._hold_far = True
                elif dist > FOLLOW_KEEP_RESUME_M:
                    self._hold_far = False
                if self._hold_far:
                    self._execute_ascii("STOP")   # 主动刹车(清零电流 → 物理 scrub 生效)
                    return
        self._execute_ascii(command)

    def _ramp_motors(self, now, blocked):
        if now - self.last_ramp >= RAMP_INTERVAL_S:
            steps = max(1, int((now - self.last_ramp) / RAMP_INTERVAL_S))
            increment = RAMP_STEP * steps
            self.last_ramp = now
            for attr, target in (("left_current", self.left_target),
                                 ("right_current", self.right_target)):
                current = getattr(self, attr)
                if current < target:
                    current = min(current + increment, target)
                elif current > target:
                    current = max(current - increment, target)
                setattr(self, attr, current)

        if now > self.command_deadline and (self.left_target or self.right_target):
            self.left_target = self.right_target = 0.0
            self.left_current = self.right_current = 0.0
            self.last_ascii = "TIMEOUT"

        if blocked and self.left_target > 0 and self.right_target > 0:
            self.left_current = self.right_current = 0.0
            self.left_target = self.right_target = 0.0
            self.last_ascii = "OBSTACLE STOP"

        scale = MAX_WHEEL_RAD_S / 255.0
        self.left_motor.setVelocity(self.left_current * scale)
        self.right_motor.setVelocity(self.right_current * scale)
        # 运动学底盘: 世界无 physics(macOS 求解器对静止/旋转车注入幽灵推力),
        # supervisor 按差速运动学积分位姿 —— C3 协议语义(斜坡/超时/DIFF)不变
        dt = self.timestep / 1000.0
        v = (self.left_current + self.right_current) / 2.0 * scale * WHEEL_RADIUS_M
        omega = (self.right_current - self.left_current) / 2.0 * scale \
            * WHEEL_RADIUS_M / WHEEL_HALF_TRACK_M
        self._kin_yaw += omega * dt
        self._kin_pos[0] -= math.sin(self._kin_yaw) * v * dt
        self._kin_pos[2] -= math.cos(self._kin_yaw) * v * dt
        self.position_field.setSFVec3f(list(self._kin_pos))
        self.rotation_field.setSFRotation([0, 1, 0, self._kin_yaw])

    def _process_key(self, key):
        if key in (ord("w"), ord("W")):
            self._manual_command("MOVE F 120")
        elif key in (ord("s"), ord("S")):
            self._manual_command("MOVE B 120")
        elif key in (ord("a"), ord("A")):
            self._manual_command("MOVE L 100")
        elif key in (ord("d"), ord("D")):
            self._manual_command("MOVE R 100")
        elif key == ord(" "):
            self.manual_until = time.monotonic() + 0.5
            self._execute_ascii("STOP")
        elif key in (ord("i"), ord("I"), ord("k"), ord("K"),
                     ord("j"), ord("J"), ord("l"), ord("L")):
            self._move_owner(key)
        elif key in (314, 315, 316, 317):   # Webots 方向键: 移动路人(双人场景)
            self._move_passerby(key)
        elif key in (ord("p"), ord("P")):
            self._toggle_passerby()
        elif key in (ord("g"), ord("G")):
            self.autopilot = not self.autopilot
            if self.autopilot:
                self.autopilot_t0 = time.monotonic()
                self._hold_far = False
                self._set_passerby(None)
                print("[SIM] demo autopilot ON — 主人自动走全场景剧本 (G 再按停止)")
            else:
                self._execute_ascii("STOP")
                print("[SIM] demo autopilot OFF")
        elif key in (ord("o"), ord("O")):
            self._toggle_bottle()
        elif key in (ord("h"), ord("H")):
            self._toggle_owner()
        elif key in (ord("t"), ord("T")):
            self.hri_enabled = not self.hri_enabled
            self.hri_action = "NONE"
            self._execute_ascii("STOP")
            print(f"[SIM] HRI {'enabled' if self.hri_enabled else 'disabled'}")
        elif key in (ord("r"), ord("R")):
            self._reset_scene()
        elif key in (ord(","), ord(".")):
            angle = 90 - math.degrees(self.pan_motor.getTargetPosition())
            adjustment = -5 if key == ord(",") else 5
            self._execute_ascii(f"PAN {clamp(angle + adjustment, 0, 180)}")
        elif key in (ord("-"), ord("=")):
            angle = 112 + math.degrees(self.tilt_motor.getTargetPosition())
            adjustment = -5 if key == ord("-") else 5
            self._execute_ascii(f"TLT {clamp(angle + adjustment, 0, 180)}")

    def _manual_command(self, command):
        self.manual_until = time.monotonic() + 0.8
        self._execute_ascii(command)

    def _move_owner(self, key):
        current = list(self.owner_translation.getSFVec3f())
        delta = 0.14
        if key in (ord("i"), ord("I")):
            current[2] = min(current[2] + delta, 3.4)
        elif key in (ord("k"), ord("K")):
            current[2] = max(current[2] - delta, -3.4)
        elif key in (ord("j"), ord("J")):
            current[0] = max(current[0] - delta, -3.4)
        else:
            current[0] = min(current[0] + delta, 3.4)
        self.owner_translation.setSFVec3f(current)
        self._sync_bottle(current)

    def _move_passerby(self, key):
        if not self.passerby_visible:
            self._set_passerby((-1.1, -1.2))
            print("[SIM] passerby shown near front")
            return
        current = list(self.passerby_translation.getSFVec3f())
        delta = 0.14
        if key == 315:      # up = 靠近
            current[2] = min(current[2] + delta, 3.4)
        elif key == 317:    # down = 远离
            current[2] = max(current[2] - delta, -3.4)
        elif key == 314:    # left
            current[0] = max(current[0] - delta, -3.4)
        elif key == 316:    # right
            current[0] = min(current[0] + delta, 3.4)
        self.passerby_translation.setSFVec3f(current)

    def _toggle_passerby(self):
        self._set_passerby(None if self.passerby_visible else (-1.1, -1.2))
        print(f"[SIM] passerby {'hidden' if not self.passerby_visible else 'shown'}")

    def _sync_bottle(self, owner_position=None):
        if not self.bottle_visible:
            return
        owner_position = owner_position or self.owner_translation.getSFVec3f()
        self.bottle_translation.setSFVec3f([
            owner_position[0] + 0.28,
            owner_position[1] + 0.88,
            owner_position[2] - 0.05,
        ])

    def _toggle_bottle(self):
        self.bottle_visible = not self.bottle_visible
        if self.bottle_visible:
            self._sync_bottle()
        else:
            self.bottle_translation.setSFVec3f(self.bottle_hidden)
        print(f"[SIM] hand object {'visible' if self.bottle_visible else 'hidden'}")

    def _toggle_owner(self):
        self.owner_visible = not self.owner_visible
        if self.owner_visible:
            self.owner_translation.setSFVec3f(self.owner_start)
            self._sync_bottle()
        else:
            self.owner_translation.setSFVec3f([100, 0, 100])
            if self.bottle_visible:
                self.bottle_translation.setSFVec3f(self.bottle_hidden)
        print(f"[SIM] owner {'visible' if self.owner_visible else 'hidden'}")

    def _reset_scene(self):
        self.autopilot = False
        self._execute_ascii("STOP")
        self._kin_pos = [0.0, 0.06, 4.5]
        self._kin_yaw = 0.49
        self.position_field.setSFVec3f(list(self._kin_pos))
        self.rotation_field.setSFRotation([0, 1, 0, self._kin_yaw])
        self.owner_visible = True
        self.owner_translation.setSFVec3f(self.owner_start)
        self.passerby_visible = False
        self.passerby_translation.setSFVec3f([100, 0, 100])
        self.bottle_visible = False
        self.bottle_translation.setSFVec3f(self.bottle_hidden)
        self.pan_motor.setPosition(0.0)
        self.tilt_motor.setPosition(0.0)
        self.left_current = self.right_current = 0.0
        self.left_target = self.right_target = 0.0
        self.hri_action = "NONE"
        self.safe_target = self.safe_target_yaw = None
        self.machine = self._new_state_machine()
        self._hold_far = False
        self.follower.stop()
        self.follower = FollowController(
            self._follow_cmd,
            person_provider=self._follow_person,
            yaw_deg_fn=self._yaw_deg,
            yaw_rate_fn=self._yaw_rate_deg,
        )
        self.follower.start()
        self.robot.simulationResetPhysics()
        print("[SIM] scene reset")

    def run(self):
        while self.robot.step(self.timestep) != -1:
            now = time.monotonic()
            if self.autotest:
                if SCENARIO == "sonar_selftest":
                    # 判别实验: 直冲端墙 30s(仿真秒), 看 sonar 是否读数 → 区分"设备坏"vs"朝向错"
                    self._execute_ascii("MOVE F 255")
                    if self.robot.getTime() > 30.0:
                        self._log({"type": "session_end", "reason": "sonar_selftest_done"})
                        self.robot.simulationQuit(0)
                        return
                elif self._demo_update(self.robot.getTime()):
                    print("[SIM] autotest done, quitting")
                    self._log({"type": "session_end", "reason": "autotest_done"})
                    self.robot.simulationQuit(0)
                    return
                self._scenario_post(self.robot.getTime())
            else:
                key = self.keyboard.getKey()
                if key > 0:
                    self._process_key(key)
                if self.autopilot:
                    if self._demo_update(time.monotonic() - self.autopilot_t0):
                        self.autopilot = False
                        self._execute_ascii("STOP")
                        print("[SIM] demo autopilot finished")

            if self.machine.state != "HIDE":
                self._hide_arrived = False
                self._drive_to_safe_target_done = False

            self._publish_owner_lock()   # 模拟 follow 进程持续发布主人锁(限频 0.2s)

            # HIDE 停靠在躲避点后持续注视主人(等效实机救援凝视; 主人去冰箱/回身递物都在视野内)
            if (GT_TURN and (not self.autotest or self.autotest_motion)
                    and self.machine.state == "HIDE" and self._hide_arrived):
                op = self.owner_translation.getSFVec3f()
                self._face_toward((op[0], op[2]))

            detections = self._detections()
            self._phantoms_update()
            # WAIT 中主人不可见(如交接后主人绕到车后): 掉头寻找主人(等效实机搜索凝视)
            # ⚠️ 这是 ground-truth 假动作(直接写 _kin_yaw); SIM_GT_TURN=0 可关, passby 必须关
            if (GT_TURN and (not self.autotest or self.autotest_motion)
                    and self.machine.state == "WAIT" and not detections
                    and self._owner_gt_dist() is not None
                    and self._owner_gt_dist() < 6.0):
                op = self.owner_translation.getSFVec3f()
                self._face_toward((op[0], op[2]))
            if self.geom_det and not detections and not self._geom_warned:
                self._geom_warned = True
                rp = self._kin_position()
                op = self.owner_translation.getSFVec3f()
                self._log({"type": "warn", "msg": "geom detector empty",
                           "robot": [round(v, 2) for v in rp],
                           "owner": [round(v, 2) for v in op]})
            distance_cm = self._virtual_sonar_cm()
            blocked = 0 < distance_cm <= SONAR_STOP_CM
            self._lidar_update(now)     # 10Hz: 虚拟雷达 → 安全包络(命令过滤在 _execute_ascii)

            if now - self.last_hri_tick >= HRI_PERIOD_S:
                self._refresh_grid()
                if self.hri_enabled:
                    # 认主回退通道暴露: owner.json 过期 + 幻影在场 → _pick_owner 将按 conf
                    # 收编幻影为"主人"(实机夜景病根的 HRI 侧残留通道), 记录暴露时机
                    if self.phantoms and self._owner_lock_stale():
                        self._log({"type": "phantom_adoption_risk",
                                   "n_phantom": len(self.phantoms),
                                   "confs": [p["conf"] for p in self.phantoms],
                                   "state": self.machine.state})
                    out = self.machine.feed(
                        detections,
                        robot_moving=(abs(self.left_current) + abs(self.right_current) > 5),
                    )
                    if now >= self.manual_until:
                        # 仿真里 follow 只在 HRI=FOLLOW 时 tick(真机 follow 线程独立于 HRI)。
                        # passby 场景例外: 主人出画会让 HRI 正确地落到 WAIT, 此时仍要继续
                        # tick, 否则"丢人→转身搜索"这条链在仿真里根本没被跑到。
                        _follow_full = ((not self.autotest or self.autotest_motion)
                                        and out["action"] == "NONE"
                                        and (self.machine.state == "FOLLOW"
                                             or (SCENARIO == "passby"
                                                 and self.machine.state == "WAIT")))
                        _parked = (self.machine.state == "WAIT"
                                   or (self.machine.state == "HIDE" and self._hide_arrived))
                        if _follow_full:
                            _ft = self.follower.tick()
                            self._last_follow_info = {
                                "cmd": None if _ft[0] is None else f"{_ft[0]} {_ft[1]}",
                                "state": (_ft[2] or {}).get("state"),
                                "phase": (_ft[2] or {}).get("phase"),
                                "theta": (_ft[2] or {}).get("theta"),
                                "rate_world": (_ft[2] or {}).get("rate_world"),
                                "turn_dir": (_ft[2] or {}).get("turn_dir"),
                            }
                        elif (_parked and not _follow_full
                              and (not self.autotest or self.autotest_motion)
                              and out["action"] == "NONE"):
                            # 停驻域(车已停好): 只允许"人从侧方过来时转头接人"——云台冻结后
                            # 相机 ±32° 是硬边界, 递物的人常站在车侧方(实测 76°), 不转头就
                            # 永远看不到人, RECEIVE 根本起不来。turn_only 不做平移/不扫 180°。
                            _ft = self.follower.tick(turn_only=True)
                            self._last_follow_info = {
                                "cmd": None if _ft[0] is None else f"{_ft[0]} {_ft[1]}",
                                "state": (_ft[2] or {}).get("state"),
                                "phase": (_ft[2] or {}).get("phase"),
                                "theta": (_ft[2] or {}).get("theta"),
                                "rate_world": (_ft[2] or {}).get("rate_world"),
                                "turn_dir": (_ft[2] or {}).get("turn_dir"),
                            }
                        elif (self.machine.state == "HIDE"
                              and self.safe_target is not None
                              and not self._hide_arrived
                              and not (self.autotest and not self.autotest_motion)):
                            self._drive_to_safe_target()
                            if self._drive_to_safe_target_done:
                                self._hide_arrived = True
                    self._log({"type": "frame", **out,
                               "dets": [d["label"] for d in detections],
                               "boxes": [[d["label"], [round(v) for v in d["bbox"]]]
                                         for d in detections],
                               "ownerbox": self._last_ownerbox,
                               "n_rec": len(self.camera.getRecognitionObjects()),
                               "owner": [round(v, 2) for v in self.owner_translation.getSFVec3f()],
                               "passerby": [round(v, 2) for v in self.passerby_translation.getSFVec3f()]
                                           if self.passerby_visible else None,
                               "robot": [round(v, 2) for v in self._kin_position()],
                               "yaw": round(self._yaw(), 3),
                               "phantoms": len(self.phantoms),
                               "sonar_cm": round(distance_cm, 1),
                               "follow": getattr(self, "_last_follow_info", None),
                               **self._lidar_log_fields()})
                self.last_hri_tick = now

            self._ramp_motors(now, blocked)
            if blocked != self.last_blocked:
                state = "BLOCKED" if blocked else "CLEAR"
                print(f"[SIM] sonar {state}: {distance_cm:.1f} cm")
                self.last_blocked = blocked

            if now - self.last_status >= 1.0:
                labels = ",".join(sorted({d["label"] for d in detections})) or "none"
                state = self.machine.state if self.hri_enabled else "OFF"
                lid = ""
                if self.lidar_on and self.lidar_env is not None:
                    lb = {k for k, v in self.lidar_env["blocked"].items() if v}
                    lid = (f" lidar={self.lidar_env['mode']}/{'|'.join(sorted(lb)) or '-'}"
                           f" gap_f={self.lidar_env['gap_m'].get('fwd')}")
                print(f"[SIM] HRI={state} owner={labels} "
                      f"sonar={distance_cm:.1f}cm cmd={self.last_ascii}{lid}")
                self.last_status = now


if __name__ == "__main__":
    ShadowCarrierSimulation().run()
