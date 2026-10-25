#!/usr/bin/env python3
"""rk_control.py v6 - USB直连C3 + 遥控/跟随模式切换 + 云台凝视(W1)"""
import os, sys, math, termios, time, threading, json, socketserver, urllib.request
from http.server import HTTPServer, BaseHTTPRequestHandler

UART = "/dev/ttyACM0"
BAUD = 115200
VIDEO_PORT = 8080
CONTROL_PORT = 80
HRI_STATE = "/dev/shm/hri_state.json"   # HRI 发布: 停驻域判断(WAIT/HIDE) → tick(turn_only)
OWNER_JSON = "/dev/shm/owner.json"      # follow 发布(只含已过换人确认门的主人框)
OWNER_DIST_MAX_AGE_S = 0.6              # 距离环允许的 owner.json 年龄
IMU_JSON = "/dev/shm/imu.json"          # imu.service 产出 (yaw_deg / yaw_rate_dps)
FOLLOW_STATUS = "/dev/shm/follow_status.json"  # 跟随时序旁路(纯增量): 供网页/记录器判"停住"


def _hri_parked(max_age=0.5, path=None):
    """HRI 是否处于停驻域(车已停好, 只应做有界转头接人)。文件缺失/过期 → False(走完整跟随)。
    path: 仅测试注入用(默认 HRI_STATE)。"""
    try:
        with open(path or HRI_STATE) as f:
            o = json.load(f)
        return (time.time() - o.get("ts", 0)) < max_age and bool(o.get("parked"))
    except (OSError, ValueError):
        return False

# 云台行程 (与 C3 固件一致, Pan 实测为 180° 舵机)
PAN_RANGE = 180.0
TILT_RANGE = 180.0
PAN_CENTER = 90.0
TILT_CENTER = 90.0
PAN_MIN, PAN_MAX = 15.0, 165.0     # 舵机指令限位(防顶死机械限位→堵转跳齿, 2026-10-03事故)
TILT_MIN, TILT_MAX = 10.0, 170.0

# 凝视控制器几何常数 (state/calib/params.py 三方互证值)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "state"))
try:
    from calib.params import TILT_LEVEL_CMD, PAN_FORWARD_CMD
except Exception:
    TILT_LEVEL_CMD = 112.0    # 兜底: 标定文件缺失时的实测值
    PAN_FORWARD_CMD = 90.58
try:
    # 单目地面几何测距 (2026-10-19): bbox 底边 → 地面距离, 走 follow 的 dist_fn 注入口。
    # 有效区间 ≈1.6–6m; 更近时脚已出画 → 返回 None(退回框高兜底 + follow 的速度门)。
    from range_from_box import ground_distance_m as _ground_distance_m
except Exception:
    _ground_distance_m = None
CONF_MIN = 0.5               # 与[认主]对齐

# 最近一次云台指令 (供步进斜坡的起步点; 所有 pan/tilt 写入都经 gimbal_send 更新)
_last_pose = {"pan": PAN_FORWARD_CMD, "tilt": TILT_LEVEL_CMD}

# A/B 测试总开关: False = 云台冻结(回到无云台的原版跟随行为)
# 2026-10-14 统筹裁定: demo 期 False —— 云台固定, 避免 _gaze_drive_loop 与 follow
# 双控制器抢底盘(往右拐/越拐越偏), 且 bearing_fn 未接线(解耦是死代码)。
GAZE_ENABLED = False

# 凝视绞合常数 (级联弧线式, arxiv 1909.06087 同款架构)
BASE_ENTER = 20.0            # 云台偏超20° → 底盘弧线转向
BASE_EXIT = 10.0             # 回到10°内 → 停止转向(滞回)
BASE_HOLD_S = 0.4            # 持续超阈确认
ARC_FAST = 140               # 弧线外侧轮速 (100→140, 重载档; 保持前进!)
ARC_SLOW = 120               # 弧线内侧轮速 (90→120, 重载档: 内轮不能低于死区)
ESC_BACK_S = 0.8             # 避障倒车时长(s)
ESC_GIVEUP_S = 3.0           # 阻碍持续多久放弃绕行改等待(s)

obs_state = {"t": 0.0, "blocked": False, "cm": -1}
obs_lock = threading.Lock()

# 世界方位角解耦(owner_bearing/bearing_fn)已【删除】(2026-10-14, 统筹"接线或删"裁定):
# 该通道从未接线, 且原公式混用"物理左正"(u_to_bearing)与"物理右正"(pan-FORWARD)两套约定
# ([世界] 22:03 审计 + world/fusion/test_bearing_convention.py 9断言)。A/B 已证 bbox 原始
# 转向够用; 云台 demo 期冻结。若将来要接线: 用 theta=(gaze.pan-PAN_FORWARD)-u_to_bearing(u),
# 且先跑 test_bearing_convention.py。

def _get_yaw_rate(path=None):
    """读 imu.json 的 yaw_rate_dps (imu.service产出)。
    数据过期(>0.5s)或质量差 → None (follow退回开环, 安全网)。path: 仅测试注入用。"""
    try:
        with open(path or IMU_JSON) as f:
            d = json.load(f)
        if time.monotonic() - float(d.get("ts", 0)) > 0.5:
            return None
        if float(d.get("quality", 1)) < 0.5:
            return None
        return float(d["yaw_rate_dps"])
    except Exception:
        return None

def _imu_zero():
    """请求 IMU 重零: 写标志文件, imu_daemon 消费后自删并重置航向。
    应在【静止】时调用 (跟随启动/停稳后)"""
    try:
        open("/dev/shm/imu_zero.req", "w").close()
        print("[imu] 已请求重零")
        return True
    except Exception:
        return False

# ====== 转身到角度 (IMU闭环, 供 [HRI] 动作层) ======
turn_state = {"active": False, "target": None, "yaw": None, "err": None,
              "started": 0.0, "result": "idle"}
turn_lock = threading.Lock()
TURN_TOL_DEG = 2.5      # 到位判定 (3→2.5: 减少过冲)
TURN_P_GAIN = 8.0       # 原地转近目标降速: spd_eff=clamp(P_GAIN*|err|, TURN_MIN_SPD, spd)
TURN_MIN_SPD = 130      # 近目标最低速度(重载下仍能维持转动)
TURN_TIMEOUT_S = 10.0   # 超时保护
TURN_DIFF = 60          # 转向差速量 (40→60, 重载档: 内轮需更大基础力矩)

def _read_yaw(path=None):
    """读积分航向 yaw_deg (逆时针为正); 过期/无效返回 None。
    path: 仅测试注入用(默认 IMU_JSON)。"""
    try:
        with open(path or IMU_JSON) as f:
            d = json.load(f)
        if time.monotonic() - float(d.get("ts", 0)) > 0.5:
            return None
        if float(d.get("quality", 1)) < 0.5:
            return None
        return float(d["yaw_deg"])
    except Exception:
        return None

# ====== 手动航向保持 (2026-10-18) ======
# 遥控只按前进时开环会持续偏航; 用 IMU yaw 做直线保持(替代固定 MOVE F)。
# 按左/右/停 → 取消保持; 下次前进重新取目标。
MANUAL_FWD_SPD = 255     # 手动前进速度(240→255 重载档; 量程255; 满电+重载下用满)
MH_K = 3.0               # yaw 误差(度) → 轮速差 增益
MH_CORR_MAX = 30         # 单轮最大修正
_mh_target = None        # 手动航向保持目标(无界 yaw_deg 的原始值)

def _wrap180(a):
    return (a + 180.0) % 360.0 - 180.0

def _manual_reset():
    global _mh_target
    _mh_target = None

def _manual_forward(send=None, yaw_fn=None):
    """遥控前进 + IMU 直线保持: DIFF 微差修正航向; IMU 不可用则退固定 MOVE F。
    send/yaw_fn: 仅测试注入用(默认 uart_send / _read_yaw)。"""
    global _mh_target
    send = send or uart_send
    y = (yaw_fn or _read_yaw)()
    if y is None:
        return send(f"MOVE F {MANUAL_FWD_SPD}")
    if _mh_target is None:
        _mh_target = y
    err = _wrap180(_mh_target - y)          # >0 需左转(逆时针)
    corr = max(-MH_CORR_MAX, min(MH_CORR_MAX, MH_K * err))
    l = int(max(0, min(255, MANUAL_FWD_SPD - corr)))
    r = int(max(0, min(255, MANUAL_FWD_SPD + corr)))
    return send(f"DIFF L{l} R{r}")

def _turn_worker(deg, spd, pivot=False):
    t0 = time.monotonic()
    try:
        y0 = _read_yaw()
        if y0 is None:      # 与 _turn_start 的 IMU 门之间的竞态窗兜底: 不 update 完整字段
            with turn_lock: # 直接判 imu_lost, 靠 finally 清预占位(不会陷 active=True)
                turn_state.update(result="imu_lost", started=t0)
            return
        with turn_lock:
            turn_state.update(active=True, target=round(y0 + deg, 2), yaw=round(y0, 2),
                              err=round(deg, 2), started=t0, result="running", pivot=pivot)
        while time.monotonic() - t0 < TURN_TIMEOUT_S:
            y = _read_yaw()
            if y is None:
                with turn_lock: turn_state["result"] = "imu_lost"
                break
            err = turn_state["target"] - y          # >0 = 需要逆时针(左转)
            with turn_lock:
                turn_state.update(yaw=round(y, 2), err=round(err, 2))
            if abs(err) < TURN_TOL_DEG:
                with turn_lock: turn_state["result"] = "ok"
                break
            if pivot:
                # 近目标比例降速: 240 PWM 在 0.1s 环里会转过 ~17°, 是 3-4° 过冲主因
                spd_eff = int(max(TURN_MIN_SPD, min(spd, TURN_P_GAIN * abs(err))))
                uart_send(f"MOVE {'L' if err > 0 else 'R'} {spd_eff}")
            else:
                hi = min(255, spd + TURN_DIFF)
                lo = max(0, spd - TURN_DIFF)
                # 逆时针(左)=左轮慢右轮快
                if err > 0:
                    uart_send(f"DIFF L{lo} R{hi}")
                else:
                    uart_send(f"DIFF L{hi} R{lo}")
            time.sleep(0.1)
        else:
            with turn_lock: turn_state["result"] = "timeout"
    finally:
        uart_send("STOP")
        with turn_lock:
            turn_state["active"] = False
        print(f"[turn] 结束: {turn_state['result']} yaw={turn_state['yaw']}"
              f"{' (pivot 原地)' if pivot else ''}")

def _turn_start(deg, spd=170, pivot=False):
    if _get_mode() != "manual":      # follow 模式由 follow 循环掌控底盘(单一控制器纪律,
        return False                 #  2026-10-14 双控制器抢底盘事故后定)
    if _read_yaw() is None:          # IMU 不可用则拒绝(宁缺毋滥)
        return False
    with turn_lock:                  # 检查+置位原子化(2026-10-25 修 TOCTOU): 旧代码无锁读
        if turn_state.get("active"): #  active, 并发双 /turn 可双双通过检查 → 双 worker 抢底盘
            return False
        turn_state["active"] = True  # 预占位; worker 启动后会 update 完整字段
    threading.Thread(target=_turn_worker, args=(deg, spd, pivot), daemon=True).start()
    return True

uart_fd = None
uart_lock = threading.Lock()
mode = "manual"
mode_lock = threading.Lock()
fc = None

def uart_open(port):
    global uart_fd
    try:
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
        import fcntl
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)  # 串口owner保护
        except BlockingIOError:
            os.close(fd)
            print(f"[UART] {port} 已被其他进程占用 (flock owner冲突)")
            return False
        attr = termios.tcgetattr(fd)
        attr[2] = attr[2] & ~(termios.CSTOPB | termios.PARENB | termios.CSIZE) | termios.CS8 | termios.CREAD | termios.CLOCAL
        attr[2] &= ~termios.CRTSCTS; attr[3] = 0
        attr[4] = termios.B115200; attr[5] = termios.B115200
        attr[6][termios.VMIN] = 0; attr[6][termios.VTIME] = 1
        termios.tcsetattr(fd, termios.TCSANOW, attr)
        import fcntl
        fl = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, fl & ~os.O_NONBLOCK)
        uart_fd = fd
        return True
    except Exception as e:
        print(f"[UART] {e}")
        return False

uart_stats = {"fail": 0, "last_fail_ts": None, "last_cmd": None, "last_err": None}
_uart_warn_t = [0.0]   # 告警节流锚点(5s 一条, 避免刷屏)

_obs_guard_veto = 0

def _obs_guard(cmd):
    """超声兜底 (2026-10-24): OBS 新鲜且 blocked 或 cm<=35 时，前进类命令即时改 STOP。
    覆盖固件只拦 MOVE F、DIFF 命令间隙继续前进的缺陷(§1.2①)；手动/跟随都生效。"""
    global _obs_guard_veto
    if not isinstance(cmd, str):
        return cmd
    with obs_lock:
        fresh = time.monotonic() - obs_state["t"] < 1.0
        blocked = bool(fresh and obs_state["blocked"])
        cm = obs_state["cm"] if fresh else None
    if not blocked and (cm is None or cm > 35):
        return cmd
    c = cmd.strip().upper()
    stop = False
    if c.startswith("MOVE F"):
        stop = True
    elif c.startswith("DIFF"):
        try:
            parts = c.split()
            l = int(parts[1].lstrip("L"))
            r = int(parts[2].lstrip("R"))
            stop = (l + r) > 0
        except Exception:
            stop = False
    if stop:
        _obs_guard_veto += 1
        return "STOP"
    return cmd

def uart_send(cmd):
    cmd = _obs_guard(cmd)
    cmd = _safety_filter(cmd)
    if cmd is None:
        return False
    with uart_lock:
        try: os.write(uart_fd, f"{cmd}\r\n".encode()); return True
        except Exception as e:
            # 2026-10-24(白名单#3): 此前裸吞异常 → C3 重枚举后本进程持死fd,
            # 所有运动命令静默失败(/ping 仍报正常), 只能人工重启。
            # 现计数入 /healthz 并节流告警; 进程内自动重连留 V2(需配udev事件测试)。
            uart_stats["fail"] += 1
            uart_stats["last_fail_ts"] = time.time()
            uart_stats["last_cmd"] = cmd
            uart_stats["last_err"] = repr(e)[:120]
            _now = time.monotonic()
            if _now - _uart_warn_t[0] >= 5.0:
                _uart_warn_t[0] = _now
                print(f"[UART] send FAIL ×{uart_stats['fail']} last={cmd!r} err={e!r}",
                      flush=True)
            return False

# ====== 激光雷达避障过滤 (V2, 2026-10-16) ======
# 设计见 docs/V2_LIDAR_AVOIDANCE.md。要点:
#   ① 惰性契约: /dev/shm/lidar_safety.json 不存在(雷达未上线/daemon 没跑) → 本函数是
#      直通, **现有链路行为逐字节不变**; 存在才生效。
#   ② 单一收口: 所有运动命令都经 uart_send → 这里, 因此 follow / HRI / 面板 / 弧线绞合
#      不需要各自知道雷达的存在。
#   ③ 过滤链自身故障(文件坏/模块缺) → 放行原命令并记一条日志, 绝不阻断控制。
SAFETY_FILE = "/dev/shm/lidar_safety.json"
SAFETY_MAX_AGE_S = 1.0          # 包络过期视为 stale → 只限速
_safety = {"t": 0.0, "env": None, "mt": None, "lg": None, "warn_t": 0.0}

def _safety_mod():
    if _safety["lg"] is None:
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        "..", "world", "fusion"))
        import lidar_guard
        _safety["lg"] = lidar_guard
    return _safety["lg"]

def _safety_env():
    """读安全包络: mtime 未变则复用(10Hz 调用无 I/O 压力); 缺失/坏 → None(直通)"""
    try:
        mt = os.stat(SAFETY_FILE).st_mtime
    except OSError:
        return None
    if _safety["mt"] == mt and time.monotonic() - _safety["t"] < 0.5:
        return _safety["env"]
    env = None
    for _ in range(2):                      # 生产者是原子替换, 容错一次并发读
        try:
            with open(SAFETY_FILE) as f:
                env = json.load(f)
            break
        except Exception:
            time.sleep(0.005)
    _safety.update(t=time.monotonic(), env=env, mt=mt)
    return env

def _safety_filter(cmd):
    """运动命令 → 安全包络过滤后命令; 非运动命令/无包络 → 原样返回"""
    env = _safety_env()
    if env is None:
        return cmd
    if time.time() - float(env.get("ts", 0)) > SAFETY_MAX_AGE_S:
        env = {"ts": env.get("ts"), "mode": "stale", "ok": False,
               "degrade_speed_mps": 0.30}
    try:
        out, note = _safety_mod().apply_to_ascii(cmd, env)
    except Exception as e:
        now = time.monotonic()
        if now - _safety["warn_t"] > 5.0:
            _safety["warn_t"] = now
            print(f"[safety] 过滤异常, 放行原命令: {e}")
        return cmd
    if note:
        now = time.monotonic()
        if now - _safety["warn_t"] > 2.0:      # 节流: 不刷屏
            _safety["warn_t"] = now
            print(f"[safety] {cmd} → {out}  ({note})")
    return out

def safety_status():
    env = _safety_env()
    if env is None:
        return {"active": False, "reason": "no safety file (雷达未上线/daemon 未运行)"}
    age = time.time() - float(env.get("ts", 0))
    return {"active": age <= SAFETY_MAX_AGE_S, "env_age_s": round(age, 2),
            "mode": env.get("mode"), "ok": env.get("ok"),
            "blocked": env.get("blocked"), "limits": env.get("limits"),
            "gap_m": env.get("gap_m"), "nearest": env.get("nearest"),
            "escape": env.get("escape"), "n_points": env.get("n_points"),
            "person_legs": env.get("person_legs")}

def _clamp_pose(v, lo, hi):
    """角度钳位 + 非有限值拒收(2026-10-25 §7.1-④): Python json.loads 默认接受 NaN/Infinity,
    "pan": NaN 经 min/max 的 nan 比较序会落到 hi(=PAN/TILT 顶死限位)。返回 None=拒收该轴。"""
    try:
        v = float(v)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(v):
        return None
    return max(lo, min(hi, v))

def gimbal_send(body):
    """body: {"pan":deg} 和/或 {"tilt":deg}, 绝对角度。
    带指令限位(防顶死) + 记录最近指令(供斜坡起步); NaN/inf 拒收该轴(宁拒勿顶死)"""
    cmds = []
    if 'pan' in body:
        p = _clamp_pose(body['pan'], PAN_MIN, PAN_MAX)
        if p is not None:
            _last_pose["pan"] = p
            cmds.append(f"PAN {p:.1f}")
    if 'tilt' in body:
        t = _clamp_pose(body['tilt'], TILT_MIN, TILT_MAX)
        if t is not None:
            _last_pose["tilt"] = t
            cmds.append(f"TLT {t:.1f}")
    if not cmds:
        return False
    ok = True
    for c in cmds:
        ok = uart_send(c) and ok
    return ok


def gimbal_ramp_to(pan, tilt, step=5.0, interval=0.06):
    """步进斜坡: 从最近指令位置以小步走向目标, 避免大跳变电流冲击/[舵机堵转跳齿]。
    2026-10-03 事故: 归中直跳导致堵转+咔咔+C3掉线; 全链路改斜坡"""
    pan = max(PAN_MIN, min(PAN_MAX, float(pan)))
    tilt = max(TILT_MIN, min(TILT_MAX, float(tilt)))

    def _run():
        for _ in range(80):                      # 上限防死循环 (~4.8s)
            dp = pan - _last_pose["pan"]
            dt = tilt - _last_pose["tilt"]
            if abs(dp) < 0.5 and abs(dt) < 0.5:
                break
            gimbal_send({
                "pan": _last_pose["pan"] + max(-step, min(step, dp)),
                "tilt": _last_pose["tilt"] + max(-step, min(step, dt)),
            })
            time.sleep(interval)

    threading.Thread(target=_run, daemon=True).start()
    return True


def gimbal_center_ramp():
    """步进归中 (替代原直跳)"""
    return gimbal_ramp_to(PAN_CENTER, TILT_CENTER)

def uart_reader():
    global uart_fd
    buf = b""
    while uart_fd is not None:
        try:
            data = os.read(uart_fd, 256)
            if data:
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    s = line.decode(errors="ignore").strip()
                    if s.startswith("OBS "):
                        try:
                            parts = s.split()
                            with obs_lock:
                                obs_state.update(t=time.monotonic(),
                                                 blocked=(parts[1] == "1"),
                                                 cm=int(parts[2]))
                        except Exception:
                            pass
            else:
                # 活 fd: VTIME=1 已阻塞 ~0.1s, 这里几乎不触发; 死 fd/EOF: os.read 会
                # 立即返回空 → 不退避则 100% 自旋(恰是 C3 掉线/重枚举场景)。
                # 10ms 防自旋退避(2026-10-25 统筹§11.1), 延迟上限 ~110ms 仍优于旧 150ms。
                time.sleep(0.01)
        except Exception:
            time.sleep(0.1)

# ====== 跟人控制 (延迟import, 失败不影响主服务) ======

def _get_obs():
    """超声波状态回调: (blocked, cm)。新鲜度与凝视驱动线程同语义(>1s 视为失效)。
    供 follow 的底盘转向通道做否决(受阻时不继续原地拧)。"""
    with obs_lock:
        fresh = time.monotonic() - obs_state["t"] < 1.0
        return (bool(fresh and obs_state["blocked"]), obs_state["cm"])

def _obs_fresh():
    """新鲜超声: (blocked, cm); 过期返回 (False, None) —— HRI 动作桥用，禁止拿旧距离做停车判据。"""
    with obs_lock:
        if time.monotonic() - obs_state["t"] < 1.0:
            return bool(obs_state["blocked"]), obs_state["cm"]
    return False, None

def _weak_cue():
    """底盘转向弱线索(owner_id 产出): 只维持原地转, 不放行前进/不进候选池。
    follow_controller.TURN_ENABLED=False 时不被消费 → 接线本身无副作用。"""
    try:
        import owner_id
        prof = fc.profile if fc is not None else None
        return owner_id.weak_cue(prof)
    except Exception:
        return None

_LEG_AREA_STATE = [None]   # (ts, area): 腿带面积快增检测


def _leg_follow_cue():
    """腿跟硬门(统筹 2026-10-22 A): mode=follow, HRI=FOLLOW, 仅1人,
    OBS 有效且 >35cm、dist>=0.7、腿带 score>=0.35。
    只返回方向 {u,score,src}; 不写 owner.json/不进 KF/不刷新模板。"""
    if _get_mode() != "follow":
        return None
    try:
        with open(HRI_STATE) as f:
            h = json.load(f)
        if time.time() - float(h.get("ts", 0)) > 0.5:
            return None
        if (h.get("state") or "") != "FOLLOW":
            return None
    except Exception:
        return None
    try:
        import owner_id
        n = 0
        for d in owner_id._fetch_raw_persons():
            if str(d.get("c")) == "person" and float(d.get("p", 0)) >= owner_id.LOWCONF_MIN_P:
                n += 1
        if n > 1:
            return None
    except Exception:
        return None
    try:
        b, cm = _get_obs()
        if b or cm is None or cm <= 0 or cm <= 35:
            return None
    except Exception:
        return None
    try:
        d = _get_owner_dist()
        if d is not None and d < 0.7:
            return None
    except Exception:
        pass
    try:
        import owner_id
        prof = fc.profile if fc is not None else None
        if prof is None:
            return None
        img = owner_id.read_recent_frame()
        if img is None:
            return None
        info = prof.find_leg_band_info(img)
        if not info:
            return None
        # 同屏只允许一个腿带候选: 多条同色竖条 → 无法判定哪条是主人 → 拒绝
        if int(info.get("n", 0)) != 1:
            print(f"[leg_follow] 腿带候选数={info.get('n')} ≠1 → 拒绝")
            return None
        if float(info["score"]) < 0.35:
            return None
        # 腿带面积快速增大 = 正在逼近(可能不是主人) → 拒绝; 0.6s 窗口
        now = time.time()
        prev = _LEG_AREA_STATE[0]
        _LEG_AREA_STATE[0] = (now, float(info.get("area", 0)))
        if prev and (now - prev[0]) <= 0.6 and prev[1] > 0:
            if float(info.get("area", 0)) > prev[1] * 1.35:
                print("[leg_follow] 腿带面积快速增大 → 拒绝(疑似逼近)")
                return None
        return {"u": float(info["u"]), "score": float(info["score"]),
                "src": "leg_band"}
    except Exception:
        return None


def _get_owner_dist():
    """主人到车的地面距离(m) —— follow 的 dist_fn 注入。
    数据源: follow 自己发布的 /dev/shm/owner.json(只含已过换人确认门的框) + 地面几何。
    返回 None 的情形(调用方自动退回框高兜底): 无文件/过期/底边被裁(<1.58m)/几何不可解。"""
    if _ground_distance_m is None:
        return None
    try:
        with open(OWNER_JSON) as f:
            o = json.load(f)
        if time.time() - float(o.get("ts", 0)) > OWNER_DIST_MAX_AGE_S:
            return None
        return _ground_distance_m(float(o["bbox"][3]), frame_h=480.0)
    except (OSError, ValueError, KeyError, TypeError):
        return None

# ====== HRI 动作桥 (A1.3, 2026-10-20) ======
# HRI 只产出 {state, action, parked}（不直接控车）；本桥是"运动控制"对动作的消费端。
# 所有动作都是有界短脉冲：动作变化/超时/障碍/IMU 丢失 → STOP。
# 桥激活期间跟随命令被丢弃，避免两个控制器抢底盘（2026-10-14 事故的纪律）。
HRI_ACTION_MAX_AGE_S = 0.5
HRI_APPROACH_SPD = 120
HRI_APPROACH_STOP_M = 0.5        # 地面测距可用时
HRI_APPROACH_STOP_CM = 35.0      # 超声兜底（<1.6m 时地面测距返回 None）
HRI_APPROACH_MAX_S = 3.0
HRI_APPROACH_GAP_S = 1.2
HRI_BACK_SPD = 120
HRI_BACK_S = 0.6
HRI_BACK_GAP_S = 1.5
HRI_GOTO_SPD = 140
HRI_GOTO_MAX_S = 1.5
HRI_GOTO_TURN_SPD = 220
HRI_GOTO_TOL_DEG = 8.0
HRI_GOTO_TURN_MAX_S = 3.0
HRI_GOTO_RETRY_S = 12.0
HRI_OBSTACLE_CM = 30.0
# grid/sector_score 的 bearing = 右正；IMU yaw 增大 = 左(CCW) → 目标 yaw = y0 - bearing
HRI_BEARING_TO_YAW_SIGN = -1.0

_HRI_LATCH = {"key": None, "t": 0.0, "pulse": -1e9}
_hri_bridge_flag = False
_hri_bridge_lock = threading.Lock()


def _hri_action(max_age=HRI_ACTION_MAX_AGE_S, path=None):
    """读 HRI 动作文件；缺失/过期/坏 JSON/无 action → None。"""
    try:
        with open(path or HRI_STATE) as f:
            o = json.load(f)
        if time.time() - float(o.get("ts", 0)) > max_age:
            return None
        return o if o.get("action") else None
    except (OSError, ValueError, TypeError):
        return None


def _hri_reset():
    """测试用：清闩锁/间隙。"""
    _HRI_LATCH.update(key=None, t=0.0, pulse=-1e9)


def _hri_bridge_active():
    with _hri_bridge_lock:
        return _hri_bridge_flag


def _hri_plan(o, now, dist=None, obs_cm=None):
    """纯决策：HRI 动作 → (kind, payload) / None。有界 + 间隙 + 闩锁。"""
    if not o:
        _HRI_LATCH["key"] = None
        return None
    act = (o.get("action") or "").strip()
    st = o.get("state") or ""
    if act.startswith("GOTO_SAFE"):
        try:
            bearing = float(act.split()[1])
        except (IndexError, ValueError):
            bearing = None
        key = f"goto:{st}:{bearing}"
        if _HRI_LATCH["key"] == key and now - _HRI_LATCH["t"] < HRI_GOTO_RETRY_S:
            return None
        _HRI_LATCH.update(key=key, t=now)
        return ("goto", bearing)
    if act == "APPROACH":
        if now - _HRI_LATCH["pulse"] < HRI_APPROACH_GAP_S:
            return None
        if dist is not None and dist <= HRI_APPROACH_STOP_M:
            return None
        if obs_cm is not None and obs_cm <= HRI_APPROACH_STOP_CM:
            return None
        _HRI_LATCH["pulse"] = now
        return ("approach", None)
    if act == "BACK_OFF":
        if now - _HRI_LATCH["pulse"] < HRI_BACK_GAP_S:
            return None
        _HRI_LATCH["pulse"] = now
        return ("back", None)
    _HRI_LATCH["key"] = None
    return None


def _hri_execute(plan, send=None, dist_fn=None, obs_fn=None, yaw_fn=None,
                 sleep_fn=time.sleep, now_fn=time.monotonic):
    """执行一个有界脉冲；send/clock 可注入（离线测试）。"""
    send = send or uart_send
    kind = plan[0]
    if kind == "back":
        send(f"MOVE B {HRI_BACK_SPD}")
        sleep_fn(HRI_BACK_S)
        send("STOP")
        return kind
    if kind == "approach":
        t0 = now_fn()
        while now_fn() - t0 < HRI_APPROACH_MAX_S:
            d = dist_fn() if dist_fn else None
            if d is not None and d <= HRI_APPROACH_STOP_M:
                break
            b, cm = obs_fn() if obs_fn else (False, None)
            if b or (cm is not None and cm <= HRI_OBSTACLE_CM):
                break
            send(f"MOVE F {HRI_APPROACH_SPD}")
            sleep_fn(0.2)
        send("STOP")
        return kind
    if kind == "goto":
        bearing = plan[1]
        y0 = yaw_fn() if yaw_fn else None
        if bearing is None or y0 is None:
            return "goto_no_ref"
        target = y0 + HRI_BEARING_TO_YAW_SIGN * bearing
        t0 = now_fn()
        while now_fn() - t0 < HRI_GOTO_TURN_MAX_S:
            y = yaw_fn()
            if y is None:
                break
            err = _wrap180(target - y)
            if abs(err) < HRI_GOTO_TOL_DEG:
                break
            send(f"MOVE {'L' if err > 0 else 'R'} {HRI_GOTO_TURN_SPD}")
            sleep_fn(0.1)
        t1 = now_fn()
        while now_fn() - t1 < HRI_GOTO_MAX_S:
            b, cm = obs_fn() if obs_fn else (False, None)
            if b or (cm is not None and cm <= HRI_OBSTACLE_CM):
                break
            send(f"MOVE F {HRI_GOTO_SPD}")
            sleep_fn(0.2)
        send("STOP")
        return kind
    return "noop"


def _hri_bridge_loop():
    """后台消费 HRI 动作；仅在 follow 模式工作。"""
    global _hri_bridge_flag
    while True:
        try:
            if _get_mode() != "follow":
                time.sleep(0.2)
                continue
            o = _hri_action()
            plan = _hri_plan(o, time.monotonic(), dist=_get_owner_dist(),
                             obs_cm=_obs_fresh()[1]) if o else None
            if plan:
                with _hri_bridge_lock:
                    _hri_bridge_flag = True
                try:
                    _hri_execute(plan, send=uart_send, dist_fn=_get_owner_dist,
                                 obs_fn=_obs_fresh, yaw_fn=_read_yaw)
                finally:
                    with _hri_bridge_lock:
                        _hri_bridge_flag = False
            time.sleep(0.1)
        except Exception as e:
            with _hri_bridge_lock:
                _hri_bridge_flag = False
            print(f"[hri] bridge error: {e}")
            time.sleep(0.2)


def _follow_send(cmd):
    """跟随命令在 HRI 动作桥激活期间丢弃（单控制器纪律）。"""
    if _hri_bridge_active():
        return False
    return uart_send(cmd)


def _publish_follow_status(state, cmd, val, info):
    """跟随时序旁路(纯增量, 不参与控制): 每拍写 /dev/shm/follow_status.json。
    供网页健康/记录器客观判"是否在驱动/停住", 不改 follow 行为。"""
    try:
        d = {"ts": round(time.time(), 3), "t": round(time.monotonic(), 3),
             "state": state, "cmd": cmd,
             "val": val if isinstance(val, (int, float, str)) else None}
        if isinstance(info, dict):
            for k in ("offset", "lost", "owner", "phase", "turn_deg", "smooth_h"):
                if k in info:
                    d[k] = info[k]
        with obs_lock:
            d["obs_blocked"] = bool(obs_state.get("blocked"))
            d["obs_cm"] = obs_state.get("cm")
        tmp = FOLLOW_STATUS + ".tmp"
        with open(tmp, "w") as f:
            json.dump(d, f)
        os.replace(tmp, FOLLOW_STATUS)
    except Exception:
        pass


def _follow_loop():
    global fc
    try:
        from follow_controller import FollowController
        fc = FollowController(_follow_send, yaw_rate_fn=_get_yaw_rate,
                              yaw_deg_fn=_read_yaw,    # 转向通道: 死推算/到位判据
                              obs_fn=_get_obs,         # 转向通道: 受阻否决
                              weak_cue_fn=_weak_cue,   # 转向通道: 人只剩腿时的方向权重
                              leg_follow_fn=_leg_follow_cue,  # 腿跟(默认关): 硬门在 rk_control
                              dist_fn=_get_owner_dist) # 距离环: 地面几何测距(替代框高)
        if not fc.start():
            # 机主原则: 认主失败拒绝跟随, 模式退回手动(凝视线随之退出)
            print("[follow] 认主失败, 跟随未启动")
            _stop_follow()
            return
        last_state = None
        while fc.running:
            try:
                if _hri_parked():
                    # 停驻域(HIDE/WAIT, 车已停好): 只允许"人从侧方过来时转头接人"
                    # 云台冻结后相机 ±32° 是硬边界; turn_only 不做平移/不扫 180°
                    _tag, _val, _info = fc.tick(turn_only=True)
                else:
                    _tag, _val, _info = fc.tick()          # 10Hz: 提速反应(原0.4s)
            except TypeError:
                _tag, _val, _info = fc.tick()              # follow 版本无 turn_only → 零行为变化
            _st = _info.get("state") if isinstance(_info, dict) else None
            _publish_follow_status(_st, _tag, _val, _info)   # 网页/记录器旁路(纯增量)
            if _st != last_state:                  # 黑匣子: 只记状态跃迁
                print(f"[follow] state {last_state} -> {_st} {_info}")
                last_state = _st
            time.sleep(0.1)
    except Exception as e:
        print(f"[follow] import/run error: {e}")
        import traceback; traceback.print_exc()
    finally:
        if fc: fc.stop()

# ====== 凝视控制器 (W1: 跟随模式云台自动盯主人) ======

gaze = None
gaze_lock = threading.Lock()

def _pick_owner_u(dets):
    """主人选择v0: conf≥CONF_MIN 的person里取框面积最大者的中心u。"""
    best = None
    for r in dets:
        if r.get("c") != "person" or r.get("p", 0) < CONF_MIN:
            continue
        area = (r["x2"] - r["x1"]) * (r["y2"] - r["y1"])
        if best is None or area > best[0]:
            best = (area, (r["x1"] + r["x2"]) / 2)
    return best[1] if best else None

def _fetch_detections():
    try:
        req = urllib.request.Request("http://127.0.0.1:8080/api/detections",
                                     headers={"User-Agent": "gaze"})
        return json.loads(urllib.request.urlopen(req, timeout=0.8).read()).get("detections", [])
    except Exception:
        return None   # None=API失败(不算丢检)

def _gaze_loop():
    """跟随模式专用线程:
    每0.1s 检测→凝视→发PAN;
    避障: 超声波BLOCKED → 倒车脱离, 持续受阻停车等待(由drive线程执行)。"""
    global gaze
    import urllib.request
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "state"))
    from gaze import GazeController
    from calib.params import TILT_LEVEL_CMD, PAN_FORWARD_CMD
    with gaze_lock:
        gaze = GazeController(PAN_FORWARD_CMD)
    gimbal_ramp_to(PAN_FORWARD_CMD, TILT_LEVEL_CMD)   # 步进入工作位(防跳变冲击舵机)
    t = time.monotonic()
    while _get_mode() == "follow":
        dets = _fetch_detections()
        if dets is not None:
            t = time.monotonic()
            u = _pick_owner_u(dets)
            out = gaze.feed(t, u)
            gimbal_send({"pan": out["pan_deg"]})
        time.sleep(0.1)

    # 退出跟随: 云台步进回中
    with gaze_lock:
        gaze = None
    gimbal_center_ramp()
    print("[gaze] 停止, 已步进回中")


def _gaze_drive_loop():
    """底盘弧线转向+避障线程 (级联式, arxiv 1909.06087 同款):
    云台指向=人的方位。云台偏 FORWARD 超20°持续0.4s → 底盘以弧线(前进+转向)
    朝人侧转; 回到10°内停止转向(滞回)。转向期间 follow 暂停(避免打架),
    弧线本身保持前进。超声波 BLOCKED 优先: 倒车脱离。"""
    corr = False
    corr_start = None
    esc_active = False
    esc_until = 0.0
    esc_start = 0.0
    while _get_mode() == "follow":
        t = time.monotonic()
        with obs_lock:
            obs_fresh = t - obs_state["t"] < 1.0
            blocked = obs_fresh and obs_state["blocked"]
        pan = gaze.pan if gaze is not None else PAN_FORWARD_CMD
        err = pan - PAN_FORWARD_CMD              # >0 = 云台指向物理右

        if blocked:
            if not esc_active:
                esc_active = True
                esc_start = t
                esc_until = t + ESC_BACK_S
                if fc is not None:
                    fc.pause()
                uart_send("STOP")
                corr = False
                corr_start = None
                print("[drive] 避障: 倒车脱离")
            if t < esc_until:
                uart_send("MOVE B 50")
            elif t - esc_start > ESC_GIVEUP_S:
                uart_send("STOP")
            time.sleep(0.1)
            continue

        if esc_active:
            esc_active = False
            uart_send("STOP")
            if fc is not None:
                fc.resume()
            print("[drive] 阻碍解除, follow恢复")

        # ---- 级联弧线转向: 云台偏哪边, 底盘往哪边弧线前进 ----
        if not corr:
            if abs(err) > BASE_ENTER:
                if corr_start is None:
                    corr_start = t
                elif t - corr_start > BASE_HOLD_S:
                    corr = True
                    corr_start = None
                    if fc is not None:
                        fc.pause()               # 暂停follow的DIFF, 弧线接管
                    print(f"[drive] 弧线转向: 云台偏{err:+.1f}°")
            else:
                corr_start = None
        else:
            if abs(err) < BASE_EXIT:
                corr = False
                if fc is not None:
                    fc.resume()
                print("[drive] 对准完成, follow恢复")
            else:
                # 云台偏右(err>0) → 底盘右转 → 左轮快右轮慢, 保持前进
                if err > 0:
                    uart_send(f"DIFF L{ARC_FAST} R{ARC_SLOW}")
                else:
                    uart_send(f"DIFF L{ARC_SLOW} R{ARC_FAST}")
        time.sleep(0.1)
    if fc is not None:
        fc.resume()
    print("[drive] 线程退出")


def _start_gaze():
    if not GAZE_ENABLED:
        print("[gaze] GAZE_ENABLED=False, 云台冻结(原版跟随模式)")
        return
    threading.Thread(target=_gaze_loop, daemon=True).start()
    threading.Thread(target=_gaze_drive_loop, daemon=True).start()

def _start_follow():
    global mode
    try:
        if fc is not None and fc.running:
            fc.resume()
            return
        with mode_lock: mode = "follow"
        _imu_zero()          # 跟随启动时车通常静止 → 重零(治冷启动标定残差)
        _start_gaze()
        threading.Thread(target=_follow_loop, daemon=True).start()
    except Exception as e:
        print(f"[follow] start error: {e}")

def _stop_follow():
    global mode
    with mode_lock: mode = "manual"
    try:
        if fc is not None:
            fc.stop()
    except Exception:
        pass
    uart_send("STOP")

# ====== 测试台 (T0b/T4) 网页支持: passby 录制 + 健康快照 ======
_passby_lock = threading.Lock()
_passby = {"thread": None, "stop": None, "path": None, "t0": 0.0,
           "n": 0, "ok": 0, "last": None, "err": None}


def _qs(path, key, default=None):
    q = path.partition('?')[2]
    for kv in q.split('&'):
        if '=' in kv:
            k, v = kv.split('=', 1)
            if k == key:
                return v
    return default


def _passby_worker(path, stop, on_sample, label=None):
    try:
        import passby_record
        passby_record.record(path, stop, rate=10.0, on_sample=on_sample, label=label)
    except Exception as e:
        with _passby_lock:
            _passby["err"] = str(e)
        print(f"[passby] recorder error: {e}")


def _passby_start(name="run", label=None):
    _prune_logs()
    with _passby_lock:
        th = _passby["thread"]
        if th is not None and th.is_alive():
            return {"ok": False, "error": "already recording", "path": _passby["path"]}
        safe = "".join(c for c in (name or "run") if (c.isalnum() or c in "._-"))[:48] or "run"
        path = os.path.join(os.path.expanduser("~kickpi/hri_logs"),
                            f"passby_{safe}_{int(time.time())}.jsonl")
        stop = threading.Event()

        def _on(rec, n, ok):
            with _passby_lock:
                _passby.update(n=n, ok=ok, last=rec)

        th = threading.Thread(target=_passby_worker, args=(path, stop, _on, label), daemon=True)
        _passby.update(thread=th, stop=stop, path=path, t0=time.monotonic(),
                       n=0, ok=0, last=None, err=None)
        th.start()
    return {"ok": True, "path": path}


def _passby_stop():
    with _passby_lock:
        th, stop, path = _passby["thread"], _passby["stop"], _passby["path"]
    if th is None or not th.is_alive():
        return {"ok": False, "error": "not recording"}
    stop.set()
    th.join(timeout=3.0)
    analysis, rc = "", -1
    try:
        import subprocess
        ap = os.path.join(os.path.dirname(os.path.abspath(__file__)), "passby_analyze.py")
        r = subprocess.run([sys.executable, ap, path], capture_output=True, text=True, timeout=20)
        analysis = (r.stdout or "") + (r.stderr or "")
        rc = r.returncode
    except Exception as e:
        analysis = f"analyzer error: {e}"
    return {"ok": True, "pass": rc == 0, "path": path, "analysis": analysis}


def _passby_status():
    with _passby_lock:
        th, n, ok, path, t0, last, err = (_passby["thread"], _passby["n"], _passby["ok"],
                                          _passby["path"], _passby["t0"], _passby["last"],
                                          _passby["err"])
    alive = bool(th is not None and th.is_alive())
    return {"running": alive, "path": path, "n": n, "owner_ok": ok,
            "vis": round(100.0 * ok / n, 1) if n else 0.0,
            "elapsed": round(time.monotonic() - t0, 1) if t0 else 0.0,
            "last": last, "error": err}


def _passby_list():
    d = os.path.expanduser("~kickpi/hri_logs")
    out = []
    try:
        for fn in sorted(os.listdir(d), reverse=True):
            if fn.startswith("passby_") and fn.endswith(".jsonl"):
                p = os.path.join(d, fn)
                st = os.stat(p)
                out.append({"name": fn, "path": p, "size": st.st_size,
                            "mtime": round(st.st_mtime, 1)})
            if len(out) >= 20:
                break
    except Exception:
        pass
    return {"files": out}


def _prune_logs(keep=200, days=30):
    """日志保留策略: 最多保留 keep 个 .jsonl, 且删除超过 days 天的。返回删除数。"""
    d = os.path.expanduser("~kickpi/hri_logs")
    removed = 0
    try:
        files = [os.path.join(d, f) for f in os.listdir(d) if f.endswith(".jsonl")]
        files.sort(key=lambda p: os.path.getmtime(p), reverse=True)
        now = time.time()
        for i, p in enumerate(files):
            if i >= keep or (now - os.path.getmtime(p)) > days * 86400:
                try:
                    os.remove(p); removed += 1
                except Exception:
                    pass
    except Exception:
        pass
    return removed


def _log_path(name):
    name = os.path.basename(name or "")
    if not name or name.startswith("."):
        return None
    d = os.path.expanduser("~kickpi/hri_logs")
    p = os.path.join(d, name)
    return p if os.path.isfile(p) else None


def _logs_list():
    d = os.path.expanduser("~kickpi/hri_logs")
    out = []
    try:
        for fn in os.listdir(d):
            if not (fn.endswith(".jsonl") or fn.endswith(".log")):
                continue
            p = os.path.join(d, fn)
            try:
                st = os.stat(p)
            except Exception:
                continue
            out.append({"name": fn, "size": st.st_size, "mtime": round(st.st_mtime, 1)})
    except Exception:
        pass
    out.sort(key=lambda x: x["mtime"], reverse=True)
    return {"dir": d, "keep": "200 个 / 30 天", "files": out[:100]}


# ====== 场景自检 (T0 链路 / T8 手动) + T1 enroll 状态 ======
_test_lock = threading.Lock()
_test = {"name": None, "thread": None, "stop": None, "steps": [],
         "running": False, "t0": 0.0, "result": None, "mode": None}

T0_STEPS = [("PING", 0.2), ("MOVE F 120", 0.6), ("MOVE L 120", 0.6)]
T8_STEPS = [("MOVE F 120", 0.4), ("MOVE B 120", 0.4),
            ("MOVE L 120", 0.4), ("MOVE R 120", 0.4), ("STOP", 0.1)]


def _test_worker(name, steps, stop):
    results = []
    try:
        for cmd, hold in steps:
            if stop.is_set():
                results.append({"cmd": cmd, "t": round(time.monotonic() - _test["t0"], 3),
                                "ok": False, "note": "aborted"})
                break
            ok = uart_send(cmd)
            results.append({"cmd": cmd, "t": round(time.monotonic() - _test["t0"], 3),
                            "ok": bool(ok)})
            with _test_lock:
                _test["steps"] = list(results)
            end = time.monotonic() + max(0.0, hold)
            while time.monotonic() < end and not stop.is_set():
                time.sleep(0.05)
            if cmd.startswith("MOVE ") and not cmd.startswith("STOP"):
                uart_send("STOP")           # 每个动作都是有界短脉冲
                results.append({"cmd": "STOP",
                                "t": round(time.monotonic() - _test["t0"], 3), "ok": True})
                with _test_lock:
                    _test["steps"] = list(results)
                time.sleep(0.15)
        if not stop.is_set():
            uart_send("STOP")
        with _test_lock:
            _test["result"] = "aborted" if stop.is_set() else "done"
    except Exception as e:
        with _test_lock:
            _test["result"] = f"error: {e}"
    finally:
        try:
            uart_send("STOP")
            _manual_reset()
        except Exception:
            pass
        with _test_lock:
            _test["running"] = False


def _test_start(name, steps, dry=False):
    with _test_lock:
        if _test["running"]:
            return {"ok": False, "error": "busy", "name": _test["name"]}
    if _get_mode() != "manual":
        _stop_follow()                     # 安全: 自检只在 manual 跑
    with _test_lock:
        _test.update(name=name, steps=[], running=not dry,
                     t0=time.monotonic(), result="dry" if dry else None,
                     mode=_get_mode())
        if dry:
            return {"ok": True, "name": name, "dry": True, "mode": _get_mode(),
                    "steps": [c for c, _ in steps]}
        stop = threading.Event()
        _test["stop"] = stop
        th = threading.Thread(target=_test_worker, args=(name, steps, stop), daemon=True)
        _test["thread"] = th
        th.start()
    return {"ok": True, "name": name, "mode": _get_mode(),
            "steps": [c for c, _ in steps]}


def _test_stop():
    with _test_lock:
        stop = _test["stop"]
    if stop is not None:
        stop.set()
    uart_send("STOP")
    return {"ok": True}


def _test_status():
    with _test_lock:
        th = _test["thread"]
        alive = bool(th is not None and th.is_alive())
        return {"name": _test["name"], "running": alive, "result": _test["result"],
                "elapsed": round(time.monotonic() - _test["t0"], 1) if _test["t0"] else 0.0,
                "steps": list(_test["steps"]), "mode": _get_mode()}


def _t1_status():
    """T1: enroll 样本数(从自身 journal 读) + owner.json 新鲜度(只读)。"""
    out = {"mode": _get_mode(), "owner_age": None, "owner_bbox": None,
           "samples": None, "template": None}
    try:
        with open(OWNER_JSON) as f:
            o = json.load(f)
        out["owner_age"] = round(time.time() - float(o.get("ts", 0)), 2)
        out["owner_bbox"] = o.get("bbox")
    except Exception:
        pass
    try:
        import subprocess
        import re
        r = subprocess.run(["journalctl", "-u", "rk-control", "-n", "500",
                            "--no-pager", "-o", "cat"],
                           capture_output=True, text=True, timeout=5)
        txt = r.stdout or ""
        m = re.findall(r"\[owner\] enroll samples=(\d+)", txt)
        if m:
            out["samples"] = int(m[-1])
        m2 = re.findall(r"\[owner\] template .*", txt)
        if m2:
            out["template"] = m2[-1].strip()
    except Exception as e:
        out["error"] = str(e)
    out["ok"] = bool(out["owner_age"] is not None and out["owner_age"] <= 1.0)
    return out


# ====== 速度/力矩现场调参 (重载档) ======
MOTION_TUNE = os.path.expanduser("~kickpi/hri_logs/motion_tune.json")
# 2026-10-25 摘除 BACK_SPD: follow_controller 里零引用(backing/back_done 是死分支)的
# 假调参键 —— 现场调了没反应却会被持久化回显, 是排障陷阱(优化审查§1-#2, 统筹批#5)。
_FC_KEYS = {"BASE_SPD", "MIN_SPD", "PIVOT_PWM", "YAW_CORR_MAX",
            "SEARCH_BACK_SPD", "MAX_OFFSET"}
_BOOL_KEYS = {"TURN_ENABLED", "LEG_FOLLOW_ENABLED"}   # T2/T3/T6/T7 基线=关转向; 腿跟默认关
_FC_FLOAT_KEYS = {"W_DES_GAIN": 5.0, "W_DES_MAX": 180.0, "YAW_K": 20.0,
                  "YAW_DEAD": 20.0, "YAW_WHEEL_MIN": 255.0}
_RK_KEYS = ("MANUAL_FWD_SPD", "HRI_APPROACH_SPD", "HRI_BACK_SPD",
            "HRI_GOTO_SPD", "HRI_GOTO_TURN_SPD", "TURN_DIFF", "ARC_FAST", "ARC_SLOW")


def _motion_get():
    try:
        import follow_controller as _fc
    except Exception:
        _fc = None
    d = {}
    if _fc is not None:
        for k in _FC_KEYS:
            d[k] = getattr(_fc, k, None)
        for k in _FC_FLOAT_KEYS:
            d[k] = getattr(_fc, k, None)
        for k in _BOOL_KEYS:
            d[k] = bool(getattr(_fc, k, False))
    for k in _RK_KEYS:
        d[k] = globals().get(k)
    d["tune_file"] = MOTION_TUNE
    return d


def _motion_set(params):
    """运行时改速度/力矩(0-255), 并持久化到 ~/hri_logs/motion_tune.json。
    follow_controller 的常量在 tick() 里按模块全局读取 → 运行中改立即生效。"""
    try:
        import follow_controller as _fc
    except Exception as e:
        return {"ok": False, "error": f"import follow_controller: {e}"}
    applied = {}
    for k, v in (params or {}).items():
        if k in _BOOL_KEYS:
            try:
                bv = bool(int(round(float(v))))
            except Exception:
                bv = bool(v)
            setattr(_fc, k, bv)
            applied[k] = bv
            continue
        if k in _FC_FLOAT_KEYS:
            try:
                fv = float(v)
            except Exception:
                continue
            fv = max(0.0, min(_FC_FLOAT_KEYS[k], fv))
            setattr(_fc, k, round(fv, 3))
            applied[k] = round(fv, 3)
            continue
        try:
            iv = int(round(float(v)))
        except Exception:
            continue
        iv = max(0, min(255, iv))
        if k in _FC_KEYS:
            setattr(_fc, k, iv)
        elif k in _RK_KEYS:
            globals()[k] = iv
        else:
            continue
        applied[k] = iv
    try:
        os.makedirs(os.path.dirname(MOTION_TUNE), exist_ok=True)
        cur = _motion_get(); cur.pop("tune_file", None); cur.update(applied)
        tmp = MOTION_TUNE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(cur, f)
        os.replace(tmp, MOTION_TUNE)
    except Exception:
        pass
    return {"ok": True, "applied": applied, "config": _motion_get()}


def _motion_load():
    try:
        with open(MOTION_TUNE) as f:
            d = json.load(f)
        if isinstance(d, dict):
            _motion_set(d)
    except Exception:
        pass


def _healthz():
    out = {"mode": _get_mode(),
           "uart": dict(uart_stats),   # 白名单#3: 串口失败计数(fail>0 且持续增长 → 死fd, 重启rk-control)
           "obs_guard_veto": _obs_guard_veto}  # 2026-10-24: 超声兜底拦截次数
    try:
        d = json.loads(urllib.request.urlopen(
            "http://127.0.0.1:8080/api/detections", timeout=1.0).read())
        out["det"] = {"fps": d.get("fps"), "age_ms": d.get("age_ms"),
                      "frames": d.get("frames"), "stats": d.get("stats", {})}
    except Exception as e:
        out["det"] = {"error": str(e)}
    try:
        with open(IMU_JSON) as f:
            imu = json.load(f)
        out["imu"] = {"age": round(time.monotonic() - float(imu.get("ts", 0)), 2),
                      "yaw": imu.get("yaw_deg"), "yaw_rate": imu.get("yaw_rate_dps"),
                      "q": imu.get("quality")}
    except Exception as e:
        out["imu"] = {"error": str(e)}
    try:
        out["camera0"] = os.readlink("/dev/camera0")
    except Exception:
        out["camera0"] = None
    try:
        with open(OWNER_JSON) as f:
            o = json.load(f)
        out["owner_age"] = round(time.time() - float(o.get("ts", 0)), 2)
    except Exception:
        out["owner_age"] = None
    try:
        with open(HRI_STATE) as f:
            h = json.load(f)
        out["hri"] = {"state": h.get("state"), "action": h.get("action"),
                      "parked": h.get("parked")}
    except Exception:
        out["hri"] = None
    try:
        with open(FOLLOW_STATUS) as f:
            fs = json.load(f)
        out["follow"] = {"age": round(time.monotonic() - float(fs.get("t", 0)), 2),
                         "state": fs.get("state"), "cmd": fs.get("cmd"),
                         "offset": fs.get("offset"), "lost": fs.get("lost")}
    except Exception:
        out["follow"] = None
    with obs_lock:
        out["obs"] = {"blocked": obs_state.get("blocked"), "cm": obs_state.get("cm"),
                      "age": round(time.monotonic() - obs_state.get("t", 0), 2)}
    try:
        import subprocess
        r = subprocess.run(["ps", "-o", "pcpu=", "-C", "dnsmasq"],
                           capture_output=True, text=True, timeout=2)
        out["dnsmasq_cpu"] = (r.stdout.strip() or "0")
    except Exception:
        out["dnsmasq_cpu"] = None
    return out


# ====== HTTP Handler ======

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def _test_page(self):
        try:
            p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test_panel.html")
            with open(p, "rb") as f:
                body = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
        except Exception as e:
            self.send_response(500)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.end_headers()
            self.wfile.write(f"test panel missing: {e}".encode())

    def do_GET(self):
        if self.path == '/test':
            self._test_page(); return
        if self.path.startswith('/passby/start'):
            self._json(_passby_start(_qs(self.path, 'name', 'run'),
                                     _qs(self.path, 'label', None))); return
        if self.path == '/passby/stop':
            self._json(_passby_stop()); return
        if self.path == '/passby/status':
            self._json(_passby_status()); return
        if self.path == '/passby/list':
            self._json(_passby_list()); return
        if self.path.startswith('/logs/list'):
            self._json(_logs_list()); return
        if self.path.startswith('/logs/get'):
            p = _log_path(_qs(self.path, 'name', ''))
            if not p:
                self.send_response(404); self.end_headers(); return
            try:
                with open(p, 'rb') as f:
                    body = f.read()
                self.send_response(200)
                self.send_header('Content-Type', 'application/octet-stream')
                self.send_header('Content-Disposition',
                                 'attachment; filename="%s"' % os.path.basename(p))
                self.send_header('Content-Length', str(len(body)))
                self.end_headers(); self.wfile.write(body)
            except Exception:
                self.send_response(500); self.end_headers()
            return
        if self.path.startswith('/logs/tail'):
            try:
                n = int(_qs(self.path, 'n', '200') or 200)
            except Exception:
                n = 200
            p = _log_path(_qs(self.path, 'name', ''))
            if not p:
                self._json({"error": "not found"}); return
            try:
                with open(p, 'r', errors='replace') as f:
                    lines = f.readlines()[-max(1, min(n, 2000)):]
                self._json({"name": os.path.basename(p),
                            "lines": [x.rstrip('\n') for x in lines]})
            except Exception as e:
                self._json({"error": str(e)})
            return
        # 安全: GET 只做 dry-run(返回步骤, 不动车); 真跑必须 POST, 且默认 dry=true
        if self.path.startswith('/test/t0'):
            self._json(_test_start("T0", T0_STEPS, dry=True)); return
        if self.path.startswith('/test/t8'):
            self._json(_test_start("T8", T8_STEPS, dry=True)); return
        if self.path.startswith('/test/stop'):
            self._json(_test_stop()); return
        if self.path.startswith('/test/status'):
            self._json(_test_status()); return
        if self.path.startswith('/test/t1'):
            self._json(_t1_status()); return
        if self.path.startswith('/motion/config'):
            self._json(_motion_get()); return
        if self.path.startswith('/healthz'):
            self._json(_healthz()); return
        if self.path == '/': self._page()
        elif self.path == '/ping': self._json({"uart": uart_fd is not None, "mode": _get_mode()})
        elif self.path == '/mode': self._json({"mode": _get_mode()})
        elif self.path == '/safety': self._json(safety_status())
        elif self.path == '/turn/status': self._json(turn_state)
        else: self.send_response(302); self.send_header('Location','/'); self.end_headers()
    def do_POST(self):
        # T0/T8 真跑必须 POST; body 缺省或 dry=true 时只返回步骤, 不动电机
        if self.path in ('/test/t0', '/test/t8'):
            try:
                cl = int(self.headers.get('Content-Length', 0))
                body = json.loads(self.rfile.read(cl)) if cl else {}
            except Exception:
                body = {}
            dry = bool(body.get('dry', True))
            if self.path.endswith('/t0'):
                self._json(_test_start("T0", T0_STEPS, dry=dry))
            else:
                self._json(_test_start("T8", T8_STEPS, dry=dry))
            return
        if self.path == '/motion/config':
            try:
                cl = int(self.headers.get('Content-Length', 0))
                body = json.loads(self.rfile.read(cl)) if cl else {}
                self._json(_motion_set(body))
            except Exception:
                self.send_response(400); self._text('BAD')
            return
        if self.path == '/move':
            # 服务端门禁(2026-10-24 审查§1.1②/白名单#1): /move 与 /forward 等同属手动运动
            # 端点。/forward~/stop 组 2026-10-16 加了"跟随时锁死", /move 当时漏了 ——
            # 是唯一能在跟随中从面板/curl 抢底盘的通道。/stop 例外语义不适用本端点。
            if _get_mode() != "manual":
                self._json({"ok": False, "error": "follow mode: 手动运动端点已锁"}); return
            try:
                cl = int(self.headers.get('Content-Length', 0))
                body = json.loads(self.rfile.read(cl)) if cl else {}
                d = body.get('direction', 'F')
                s = max(0, min(255, int(body.get('speed', 150))))
                if d in 'FBLR' and len(d) == 1:
                    self._text("OK" if uart_send(f"MOVE {d} {s}") else "FAIL")
                    return
            except Exception: pass
            self.send_response(400); self._text('BAD')
            return
        if self.path == '/gimbal':
            try:
                cl = int(self.headers.get('Content-Length', 0))
                body = json.loads(self.rfile.read(cl)) if cl else {}
                if gimbal_send(body):
                    self._text("OK"); return
            except Exception: pass
            self.send_response(400); self._text('BAD')
            return
        if self.path == '/gimbal/center':
            ok = gimbal_center_ramp()
            self._text("OK" if ok else "FAIL")
            return
        if self.path == '/turn':
            try:
                cl = int(self.headers.get('Content-Length', 0))
                body = json.loads(self.rfile.read(cl)) if cl else {}
                deg = float(body.get('deg', 0))
                spd = int(body.get('speed', 170))
                if _turn_start(deg, spd):
                    self._json({"ok": True, "state": turn_state})
                else:
                    self._json({"ok": False, "state": turn_state,
                                "error": "busy / imu unavailable / follow mode"})
                return
            except Exception:
                self.send_response(400); self._text('BAD')
                return
        if self.path == '/imu/zero':
            self._json({"ok": _imu_zero()})
            return
        if self.path == '/turn_pivot':
            # 原地对转(真·原地掉头)到指定角度: IMU 闭环。手动模式专用 —— 跟随态的
            # 原地转由 follow_controller 的转向通道内部完成(同一控制器纪律)。
            try:
                cl = int(self.headers.get('Content-Length', 0))
                body = json.loads(self.rfile.read(cl)) if cl else {}
                deg = float(body.get('deg', 180))
                try:      # 速度默认值与转向通道同源(保持延迟 import 的纪律)
                    from follow_controller import PIVOT_PWM as _pivot_pwm
                except Exception:
                    _pivot_pwm = 110
                spd = int(body.get('speed', _pivot_pwm))
                if _turn_start(deg, spd, pivot=True):
                    self._json({"ok": True, "state": turn_state})
                else:
                    self._json({"ok": False, "state": turn_state,
                                "error": "busy / imu unavailable / follow mode"})
                return
            except Exception:
                self.send_response(400); self._text('BAD')
                return
        r = {'/forward':None, '/back':'MOVE B 240', '/left':'MOVE L 190',
             '/right':'MOVE R 190', '/stop':'STOP'}
        if self.path in r:
            # 服务端门禁(2026-10-16): 跟随时底盘归 follow 循环独占, 面板/curl 误触会抢控制。
            # /stop 例外 —— 任何模式下都必须能急停。
            if self.path != "/stop" and _get_mode() != "manual":
                self._json({"ok": False, "error": "follow mode: 手动运动端点已锁"})
                return
            if self.path == '/forward':
                _manual_forward()            # IMU 直线保持(替代固定 MOVE F)
            else:
                if self.path in ('/back', '/left', '/right', '/stop'):
                    _manual_reset()          # 转向/停/后退 → 取消航向保持
                uart_send(r[self.path])
            self._text("OK")
        elif self.path == '/mode/manual':
            _stop_follow()
            self._json({"mode": "manual"})
        elif self.path == '/mode/follow':
            _start_follow()
            self._json({"mode": "follow"})
        else: self.send_response(404); self._text('404')
    def _text(self, m):
        self.send_response(200); self.send_header('Content-Type','text/plain'); self.end_headers(); self.wfile.write(m.encode())
    def _json(self, d):
        self.send_response(200); self.send_header('Content-Type','application/json'); self.end_headers(); self.wfile.write(json.dumps(d).encode())
    def _page(self):
        vp = VIDEO_PORT
        html = f'''<!DOCTYPE html><html lang="zh"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>ShadowCarrier-RK</title>
<style>
*{{margin:0;padding:0;box-sizing:border-box}}
body{{background:#0d1117;color:#c9d1d9;font-family:-apple-system,sans-serif;height:100dvh;display:flex;flex-direction:column}}
.video{{flex:1;display:flex;align-items:center;justify-content:center;background:#000;min-height:0}}
.video img{{max-width:100%;max-height:100%;object-fit:contain}}
.modebar{{display:flex;padding:12px 12px 0;gap:0}}
.modebar button{{flex:1;height:44px;border:0;font-size:17px;font-weight:700;color:#fff;cursor:pointer}}
.modebar button:first-child{{border-radius:12px 0 0 12px}}
.modebar button:last-child{{border-radius:0 12px 12px 0}}
.modebar button.active{{background:#238636}}
.modebar button.inactive{{background:#21262d;color:#8b949e}}
.ctrl{{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;padding:12px;background:#161b22}}
.gimbal{{display:flex;gap:10px;padding:0 12px 12px;background:#161b22;align-items:center}}
.pad{{flex:1;height:96px;background:#21262d;border:1px solid #30363d;border-radius:12px;position:relative;touch-action:none;overflow:hidden}}
.knob{{position:absolute;left:50%;top:50%;width:26px;height:26px;margin:-13px 0 0 -13px;background:#238636;border-radius:50%;pointer-events:none;opacity:.9;transition:transform .08s}}
.padhint{{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;color:#6e7681;font-size:13px;pointer-events:none}}
.gbtn{{height:96px;width:76px;font-size:15px;background:#30363d;line-height:1.3}}
button{{border:0;border-radius:10px;font-size:20px;font-weight:700;height:64px;touch-action:none;user-select:none;color:#fff}}
button:active{{transform:scale(.96);filter:brightness(.85)}}
.fwd{{background:#238636;grid-column:2}}.bck{{background:#1f6feb;grid-column:2}}
.left{{background:#d29922;grid-column:1;grid-row:2}}
.stop{{background:#da3633;grid-column:2;grid-row:2;font-size:28px}}
.right{{background:#d29922;grid-column:3;grid-row:2}}
.gbtn{{height:48px;font-size:18px;background:#30363d}}
button:disabled{{opacity:.3}}
.status{{padding:8px 12px;background:#0d1117;color:#8b949e;font-size:13px;text-align:center}}
</style></head><body>
<div class="modebar">
<button id="btnManual" class="active" onclick="setMode('manual')">遥控</button>
<button id="btnFollow" class="inactive" onclick="setMode('follow')">跟随</button>
</div>
<div class="video"><img id="vfeed"></div>
<div class="ctrl" id="ctrlPad">
<button class="fwd" data-cmd="forward">▲</button>
<button class="left" data-cmd="left">◀</button>
<button class="stop" data-cmd="stop">■</button>
<button class="right" data-cmd="right">▶</button>
<button class="bck" data-cmd="back">▼</button>
</div>
<div class="gimbal">
<div class="pad" id="gpad"><div class="knob" id="gknob"></div><span class="padhint">拖动云台</span></div>
<button class="gbtn" id="gcenter">◎<br>回中</button>
</div><div class="status" id="s">USB直连 C3 | 遥控模式</div>
<div style="padding:6px 12px 10px;text-align:center"><a href="/test" style="color:#58a6ff;font-size:15px;text-decoration:none">🧪 测试台（T0b / T4 / 健康）</a></div>
<script>
let curMode='manual',t=null,a=null;
function s(c,l){{fetch('/'+c,{{method:'POST'}}).then(r=>r.text()).then(x=>{{document.getElementById('s').textContent=l+': '+x}}).catch(e=>document.getElementById('s').textContent='ERR')}}
function h(c,l){{if(curMode!=='manual')return;if(t)clearInterval(t);a=c;s(c,l);t=setInterval(()=>s(c,l),150)}}
function r(){{if(t){{clearInterval(t);t=null}}if(a){{s('stop','STOP');a=null}}}}
let gt=null,gpos={{pan:{PAN_CENTER},tilt:{TILT_CENTER}}};
function g(d){{
if(curMode!=='manual')return;
if(d==='center'){{gpos={{pan:{PAN_CENTER},tilt:{TILT_CENTER}}};}}
else{{
 gpos.pan=Math.max(0,Math.min({PAN_RANGE},gpos.pan+(d.pan||0)));
 gpos.tilt=Math.max(0,Math.min({TILT_RANGE},gpos.tilt+(d.tilt||0)));
}}
fetch('/gimbal',{{method:'POST',headers:{{'Content-Type':'application/json'}},body:JSON.stringify(gpos)}})
.then(r=>r.text()).then(x=>{{document.getElementById('s').textContent='PAN '+gpos.pan.toFixed(0)+' | TILT '+gpos.tilt.toFixed(0)+' | '+x}})
.catch(e=>document.getElementById('s').textContent='ERR');
}}
const pad=document.getElementById('gpad'),knob=document.getElementById('gknob');
let dragging=false,ox=0,oy=0,kx=0,ky=0;
setInterval(()=>{{if(dragging&&curMode==='manual')g({{pan:kx*0.12,tilt:ky*0.12}})}},100);
document.getElementById('gcenter').addEventListener('pointerdown',e=>{{e.preventDefault();centerRamp()}});
function centerRamp(){{
 // 回中改为小步渐变(复用拖拽路径): 避免两舵机同时大跳导致电流尖峰/过冲堵转
 if(curMode!=='manual')return;
 let i=0;
 const timer=setInterval(()=>{{
  if(curMode!=='manual'){{clearInterval(timer);return;}}
  const dp=Math.sign({PAN_CENTER}-gpos.pan)*Math.min(5,Math.abs({PAN_CENTER}-gpos.pan));
  const dt=Math.sign({TILT_CENTER}-gpos.tilt)*Math.min(5,Math.abs({TILT_CENTER}-gpos.tilt));
  if(dp===0&&dt===0){{clearInterval(timer);return;}}
  g({{pan:dp,tilt:dt}});
  if(++i>=40)clearInterval(timer);
 }},120);
}}
pad.addEventListener('pointerdown',e=>{{e.preventDefault();if(curMode!=='manual')return;dragging=true;ox=e.clientX-kx;oy=e.clientY-ky;pad.setPointerCapture(e.pointerId)}});
pad.addEventListener('pointermove',e=>{{if(!dragging)return;kx=Math.max(-40,Math.min(40,e.clientX-ox));ky=Math.max(-40,Math.min(40,e.clientY-oy));knob.style.transform='translate('+kx+'px,'+ky+'px)'}});
function endDrag(){{dragging=false;kx=0;ky=0;knob.style.transform='translate(0,0)'}}
pad.addEventListener('pointerup',endDrag);pad.addEventListener('pointercancel',endDrag);pad.addEventListener('lostpointercapture',endDrag);
window.addEventListener('blur',endDrag);
function setMode(m){{
if(m===curMode)return;
fetch('/mode/'+m,{{method:'POST'}}).then(r=>r.json()).then(d=>{{
 curMode=d.mode;
 document.getElementById('btnManual').className=curMode==='manual'?'active':'inactive';
 document.getElementById('btnFollow').className=curMode==='follow'?'active':'inactive';
 document.getElementById('s').textContent='USB直连 C3 | '+(curMode==='manual'?'遥控模式':'跟随模式');
 let btns=document.querySelectorAll('.ctrl button,.gbtn');
 btns.forEach(b=>{{if(b.dataset.cmd!=='stop')b.disabled=curMode!=='manual';}});
 if(curMode!=='manual'){{
   r();
   gpos={{pan:{PAN_CENTER},tilt:{TILT_CENTER}}};
   fetch('/gimbal/center',{{method:'POST'}});
 }}
}}).catch(e=>console.log(e));}}
document.querySelectorAll('.ctrl button').forEach(b=>{{
 b.addEventListener('contextmenu',e=>e.preventDefault());
 if(b.dataset.cmd==='stop'){{
  b.addEventListener('pointerdown',e=>{{
   e.preventDefault();
   if(curMode==='follow'){{setMode('manual');return;}}
   r();
  }});return;
 }}
 b.addEventListener('pointerdown',e=>{{e.preventDefault();b.setPointerCapture(e.pointerId);h(b.dataset.cmd,b.textContent.trim())}});
 b.addEventListener('pointerup',r);b.addEventListener('pointercancel',r);b.addEventListener('lostpointercapture',r);
}});
window.addEventListener('pointerup',r);window.addEventListener('blur',r);
window.onload=function(){{setTimeout(function(){{document.getElementById('vfeed').src='http://'+location.hostname+':{vp}/stream';}},500);}};
</script></body></html>'''
        self.send_response(200); self.send_header('Content-Type','text/html; charset=utf-8'); self.end_headers(); self.wfile.write(html.encode())

class TServer(socketserver.ThreadingMixIn, HTTPServer):
    allow_reuse_address = True; daemon_threads = True

def _get_mode():
    with mode_lock: return mode

def main():
    global UART
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('--port', type=int, default=CONTROL_PORT)
    p.add_argument('--uart', type=str, default=UART)
    args = p.parse_args()
    UART = args.uart
    print(f"ShadowCarrier-RK v5 | HTTP :{args.port} | USB->C3 {UART}")
    ok = uart_open(UART)
    if ok: time.sleep(0.5)
    print("OK" if ok else "UART FAIL (will retry on udev)")
    _motion_load()          # 重载档现场调参(~/hri_logs/motion_tune.json), 启动时恢复
    threading.Thread(target=uart_reader, daemon=True).start()
    threading.Thread(target=_hri_bridge_loop, daemon=True).start()
    httpd = TServer(('0.0.0.0', args.port), Handler)
    try: httpd.serve_forever()
    except KeyboardInterrupt: pass
    finally: httpd.shutdown(); os.close(uart_fd) if uart_fd else None

if __name__ == '__main__': main()
