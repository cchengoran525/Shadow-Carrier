#!/usr/bin/env python3
"""Webots adapter for the current C3 ASCII protocol and RK-side HRI logic."""
import json
import math
import os
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
# 走廊 demo 场景常量 (世界坐标 x, z)
DORM_DOOR_POS = (-0.85, 3.0)        # 宿舍门内侧(路人入场点)
HIDE_SPOT = (1.0, -4.3)             # 饮水机旁墙根: 离门 ~9m, 不挡饮水机/冰箱正前方
FOLLOW_KEEP_MIN_M = 2.6             # 仿真车距保持: 近于此距离刹车暂停跟随
FOLLOW_KEEP_RESUME_M = 3.0          # 远于此距离恢复推进(滞回)


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
        self._log_open()
        self.machine = self._new_state_machine()
        self.follower = FollowController(
            self._follow_cmd,
            person_provider=self._follow_person,
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
        path = Path(__file__).resolve().parent / "sim_session.jsonl"
        self.log_file = open(path, "w", buffering=1, encoding="utf-8")  # 每次运行覆盖, 避免多会话混在一起
        self.log_t0 = time.monotonic()
        self.log_file.write(json.dumps({"type": "session_start",
                                        "t0": time.time(),
                                        "geom_det": self.geom_det,
                                        "autotest": self.autotest,
                                        "fx_px": round(self.fx_px, 1)}) + "\n")
        print(f"[SIM] 黑匣子: {path}")

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
        ) if o]
        # 自由方向 = 场景躲避点(HIDE_SPOT)相对车的方位角(仿真实机滚动地图的自由扇区)
        free = []
        rp = self._kin_position()
        yaw = self._yaw()
        dx, dz = HIDE_SPOT[0] - rp[0], HIDE_SPOT[1] - rp[2]
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

    def _synthetic_box(self, node, w_m, h_m):
        """3D 位置 + 针孔模型合成检测框; 深度窗外返回 None。
        用车体真实位姿投影(运动模式必须); 观察模式车被钉在原点, 结果与旧假定原点一致。
        横向按 lx/depth 投影像素偏移, 双人场景两个框才能真正分开。"""
        pos = node.getField("translation").getSFVec3f()
        rp = self._kin_position()
        yaw = self._yaw()
        dx, dz = pos[0] - rp[0], pos[2] - rp[2]
        lx = math.cos(yaw) * dx - math.sin(yaw) * dz
        lz = math.sin(yaw) * dx + math.cos(yaw) * dz
        depth = -lz   # 车前方为 -z
        if depth < 0.15 or depth > 6.0:
            return None
        bw = self.fx_px * w_m / depth
        bh = self.fx_px * h_m / depth
        cx = self.camera.getWidth() / 2 + self.fx_px * (lx / depth)
        cy = self.camera.getHeight() / 2
        return [cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2]

    def _geom_detections(self):
        """几何真值检测: 主人 / 手持物 / 路人(可见时)"""
        out = []
        for label, node, w_m, h_m in (
                ("person", self.owner_node, self.PERSON_W, self.PERSON_H),
                ("bottle", self.bottle_node, 0.09, 0.22),
                ("person", self.passerby_node, self.PERSON_W, self.PERSON_H)):
            if node is None:
                continue
            if node is self.passerby_node and not self.passerby_visible:
                continue
            bbox = self._synthetic_box(node, w_m, h_m)
            if bbox is None:
                continue
            out.append({"label": label, "conf": 0.95, "bbox": bbox})
        return out

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
                angle = clamp(float(parts[1]), 0.0, 180.0)
            except ValueError:
                return
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
        for spot in (HIDE_SPOT,):
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
        )
        self.follower.start()
        self.robot.simulationResetPhysics()
        print("[SIM] scene reset")

    def run(self):
        while self.robot.step(self.timestep) != -1:
            now = time.monotonic()
            if self.autotest:
                if self._demo_update(self.robot.getTime()):
                    print("[SIM] autotest done, quitting")
                    self._log({"type": "session_end", "reason": "autotest_done"})
                    self.robot.simulationQuit(0)
                    return
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
            if ((not self.autotest or self.autotest_motion)
                    and self.machine.state == "HIDE" and self._hide_arrived):
                op = self.owner_translation.getSFVec3f()
                self._face_toward((op[0], op[2]))

            detections = self._detections()
            # WAIT 中主人不可见(如交接后主人绕到车后): 掉头寻找主人(等效实机搜索凝视)
            if ((not self.autotest or self.autotest_motion)
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
            distance_cm = float(self.sonar.getValue())
            blocked = 0 < distance_cm <= SONAR_STOP_CM

            if now - self.last_hri_tick >= HRI_PERIOD_S:
                self._refresh_grid()
                if self.hri_enabled:
                    out = self.machine.feed(
                        detections,
                        robot_moving=(abs(self.left_current) + abs(self.right_current) > 5),
                    )
                    if now >= self.manual_until:
                        if ((not self.autotest or self.autotest_motion)
                                and out["action"] == "NONE"
                                and self.machine.state == "FOLLOW"):
                            self.follower.tick()
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
                               "sonar_cm": round(float(self.sonar.getValue()), 1)})
                self.last_hri_tick = now

            self._ramp_motors(now, blocked)
            if blocked != self.last_blocked:
                state = "BLOCKED" if blocked else "CLEAR"
                print(f"[SIM] sonar {state}: {distance_cm:.1f} cm")
                self.last_blocked = blocked

            if now - self.last_status >= 1.0:
                labels = ",".join(sorted({d["label"] for d in detections})) or "none"
                state = self.machine.state if self.hri_enabled else "OFF"
                print(f"[SIM] HRI={state} owner={labels} "
                      f"sonar={distance_cm:.1f}cm cmd={self.last_ascii}")
                self.last_status = now


if __name__ == "__main__":
    ShadowCarrierSimulation().run()
