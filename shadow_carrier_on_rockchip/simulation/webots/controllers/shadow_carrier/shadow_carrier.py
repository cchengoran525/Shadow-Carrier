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
import hri_state
from follow_controller import FollowController


TIMEOUT_S = 0.45
RAMP_INTERVAL_S = 0.02
RAMP_STEP = 8
MAX_WHEEL_RAD_S = 10.0
SONAR_STOP_CM = 20.0
HRI_PERIOD_S = 0.30


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
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.pan_motor = self.robot.getDevice("pan motor")
        self.tilt_motor = self.robot.getDevice("tilt motor")
        self.pan_motor.setPosition(0.0)
        self.tilt_motor.setPosition(0.0)

        self.camera = self.robot.getDevice("front camera")
        self.camera.enable(self.timestep)
        self.camera.recognitionEnable(self.timestep)
        self.sonar = self.robot.getDevice("front sonar")
        self.sonar.enable(self.timestep)
        self.keyboard = self.robot.getKeyboard()
        self.keyboard.enable(self.timestep)

        self.owner_node = self.robot.getFromDef("OWNER")
        self.owner_translation = self.owner_node.getField("translation")
        self.owner_start = list(self.owner_translation.getSFVec3f())
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

        self.grid_path = Path(tempfile.gettempdir()) / "shadow_carrier_webots_grid.json"
        self._log_open()
        self.machine = self._new_state_machine()
        self.follower = FollowController(
            self._execute_ascii,
            person_provider=self._follow_person,
        )
        self.follower.start()
        print("[SIM] Webots adapter ready. HRI is on; use T to toggle it.")
        print("[SIM] WASD drive, Space stop; I/K/J/L move owner; O offer object; H hide owner; R reset.")
        print("[SIM] ',' '.' pan; '-' '=' tilt. Commands follow the C3 450 ms timeout and ramp.")
        self.autotest = bool(os.environ.get("SIM_AUTOTEST"))
        self.autotest_motion = os.environ.get("SIM_AUTOTEST_MOTION") == "1"
        if self.autotest:
            print("[SIM] AUTOTEST mode: scripted owner timeline, keyboard ignored, auto-quit at end.")
            print(f"[SIM] chassis motion: {'ON' if self.autotest_motion else 'OFF (observe-only)'}")

    def _set_person(self, z):
        self.owner_node.getField("translation").setSFVec3f([0, 0, z])

    def _set_object(self, offered, z):
        if offered:
            self.bottle_node.getField("translation").setSFVec3f([0.30, 0.95, z])
        else:
            self.bottle_node.getField("translation").setSFVec3f([100, 0.9, 100])

    def _autotest_update(self, t):
        """脚本驱动: 静止->缓慢靠近(持物)->后退->快冲->静止(HIDE)->持物靠近->消失. 返回 True 结束."""
        if t < 16:            # 静止 2.5m: FOLLOW -> WAIT(5s)
            self._set_person(-2.5); self._set_object(False, 0)
        elif t < 24:          # 持物缓慢靠近: RECEIVE
            z = -2.5 + (t - 16) / 8.0 * 1.3
            self._set_person(z); self._set_object(True, z)
        elif t < 30:          # 后退: RECEIVE -> WAIT
            z = -1.2 - (t - 24) / 6.0 * 1.0
            self._set_person(z); self._set_object(True, z)
        elif t < 31.5:        # 快冲(~0.93m/s): YIELD
            z = -2.2 + (t - 30) / 1.5 * 1.4
            self._set_person(z); self._set_object(False, 0)
        elif t < 40:          # 停在近处
            self._set_person(-0.8); self._set_object(False, 0)
        elif t < 42:          # 快速退开
            z = -0.8 - (t - 40) / 2.0 * 1.7
            self._set_person(z); self._set_object(False, 0)
        elif t < 68:          # 静止 -> WAIT -> HIDE(20s) -> GOTO_SAFE
            self._set_person(-2.5); self._set_object(False, 0)
        elif t < 76:          # 持物缓慢靠近: RECEIVE(持物)
            z = -2.5 + (t - 68) / 8.0 * 1.3
            self._set_person(z); self._set_object(True, z)
        elif t < 84:          # 主人消失: owner-lost / wait_owner
            self._set_person(100); self._set_object(False, 0)
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
        return hri_state.HRIStateMachine(
            send_cmd_fn=self._on_hri_action,
            log_fn=self._hri_log,
            params=params,
        )

    def _log_open(self):
        path = Path(__file__).resolve().parent / "sim_session.jsonl"
        self.log_file = open(path, "w", buffering=1)  # 每次运行覆盖, 避免多会话混在一起
        self.log_t0 = time.monotonic()
        self.log_file.write(json.dumps({"type": "session_start",
                                        "t0": time.time()}) + "\n")
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
        rotation = self.self_node.getOrientation()
        return math.atan2(rotation[2], rotation[0])

    def _bearing_to(self, target):
        position = self.self_node.getPosition()
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
        robot_pos = self.self_node.getPosition()
        dist = math.hypot(pos[0] - robot_pos[0], pos[2] - robot_pos[2])
        return {"cls": cls, "bearing_deg": self._bearing_to(pos),
                "dist_m": round(dist, 2)}

    def _refresh_grid(self):
        """填 demo 全场景的语义快照: 门 + 景点(饮水机/冰箱/柜台)"""
        objects = [o for o in (
            self._grid_object("SAFE_DOOR", "door"),
            self._grid_object("WATER_DISPENSER", "refrigerator"),
            self._grid_object("REFRIGERATOR", "refrigerator"),
            self._grid_object("COUNTER", "dining table"),
        ) if o]
        snapshot = {
            "ts": time.time(),
            "objects": objects,
            "free_directions_deg": [-45, 45],
        }
        try:
            self.grid_path.write_text(json.dumps(snapshot), encoding="utf-8")
        except OSError as error:
            print(f"[SIM] grid snapshot unavailable: {error}")

    def _detections(self):
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

    def _follow_person(self):
        person = next((d for d in self._detections() if d["label"] == "person"), None)
        if person is None:
            return None
        x1, y1, x2, y2 = person["bbox"]
        return {
            "cx": (x1 + x2) / 2,
            "cy": (y1 + y2) / 2,
            "h": y2 - y1,
        }

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
                self._execute_ascii("MOVE F 45")
            return
        self.hri_action = action
        if action == "NONE":
            self.safe_target = None
            self.safe_target_yaw = None
            self._execute_ascii("STOP")
        elif action == "BACK_OFF":
            self.safe_target = None
            self.safe_target_yaw = None
            self._execute_ascii("MOVE B 60")
        elif action == "APPROACH":
            self.safe_target = None
            self.safe_target_yaw = None
            self._execute_ascii("MOVE F 45")
        elif action.startswith("GOTO_SAFE"):
            match = re.search(r"GOTO_SAFE\s+(-?\d+(?:\.\d+)?)", action)
            bearing = float(match.group(1)) if match else 0.0
            yaw_target = self._yaw() - math.radians(bearing)
            position = self.self_node.getPosition()
            self.safe_target_yaw = yaw_target
            self.safe_target = [
                position[0] - math.sin(yaw_target),
                position[1],
                position[2] - math.cos(yaw_target),
            ]

    def _drive_to_safe_target(self):
        if self.safe_target is None:
            return
        position = self.self_node.getPosition()
        remaining = math.hypot(self.safe_target[0] - position[0],
                               self.safe_target[2] - position[2])
        if remaining <= 0.12:
            self._execute_ascii("STOP")
            return
        yaw_error = (self.safe_target_yaw - self._yaw() + math.pi) % (2 * math.pi) - math.pi
        if abs(yaw_error) > math.radians(8):
            self._execute_ascii("MOVE L 42" if yaw_error > 0 else "MOVE R 42")
        else:
            self._execute_ascii("MOVE F 55")

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
        self._execute_ascii("STOP")
        self.position_field.setSFVec3f([0, 0.06, 0])
        self.rotation_field.setSFRotation([0, 1, 0, 0])
        self.owner_visible = True
        self.owner_translation.setSFVec3f(self.owner_start)
        self.bottle_visible = False
        self.bottle_translation.setSFVec3f(self.bottle_hidden)
        self.pan_motor.setPosition(0.0)
        self.tilt_motor.setPosition(0.0)
        self.left_current = self.right_current = 0.0
        self.left_target = self.right_target = 0.0
        self.hri_action = "NONE"
        self.safe_target = self.safe_target_yaw = None
        self.machine = self._new_state_machine()
        self.follower.stop()
        self.follower = FollowController(
            self._execute_ascii,
            person_provider=self._follow_person,
        )
        self.follower.start()
        self.robot.simulationResetPhysics()
        print("[SIM] scene reset")

    def run(self):
        while self.robot.step(self.timestep) != -1:
            now = time.monotonic()
            if self.autotest:
                if self._autotest_update(self.robot.getTime()):
                    print("[SIM] autotest done, quitting")
                    self._log({"type": "session_end", "reason": "autotest_done"})
                    self.robot.simulationQuit(0)
                    return
            else:
                key = self.keyboard.getKey()
                if key > 0:
                    self._process_key(key)

            detections = self._detections()
            distance_cm = float(self.sonar.getValue())
            blocked = 0 < distance_cm <= SONAR_STOP_CM

            if now - self.last_hri_tick >= HRI_PERIOD_S:
                self._refresh_grid()
                if self.hri_enabled:
                    out = self.machine.feed(
                        detections,
                        robot_moving=(abs(self.left_current) + abs(self.right_current) > 5),
                    )
                    if now >= self.manual_until and not (self.autotest and not self.autotest_motion):
                        if out["action"] == "NONE" and self.machine.state == "FOLLOW":
                            self.follower.tick()
                        elif out["action"].startswith("GOTO_SAFE"):
                            self._drive_to_safe_target()
                    self._log({"type": "frame", **out,
                               "dets": [d["label"] for d in detections],
                               "boxes": {d["label"]: [int(v) for v in d["bbox"]] for d in detections},
                               "n_rec": len(self.camera.getRecognitionObjects()),
                               "owner": [round(v, 2) for v in self.owner_translation.getSFVec3f()],
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
