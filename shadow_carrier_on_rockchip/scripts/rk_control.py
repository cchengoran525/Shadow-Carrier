#!/usr/bin/env python3
"""rk_control.py v6 - USB直连C3 + 遥控/跟随模式切换 + 云台凝视(W1)"""
import os, sys, termios, time, threading, json, socketserver, urllib.request
from http.server import HTTPServer, BaseHTTPRequestHandler

UART = "/dev/ttyACM0"
BAUD = 115200
VIDEO_PORT = 8080
CONTROL_PORT = 80

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
CONF_MIN = 0.5               # 与[认主]对齐

# 最近一次云台指令 (供步进斜坡的起步点; 所有 pan/tilt 写入都经 gimbal_send 更新)
_last_pose = {"pan": PAN_FORWARD_CMD, "tilt": TILT_LEVEL_CMD}

# A/B 测试总开关: False = 云台冻结(回到无云台的原版跟随行为)
GAZE_ENABLED = True

# 凝视绞合常数 (级联弧线式, arxiv 1909.06087 同款架构)
BASE_ENTER = 20.0            # 云台偏超20° → 底盘弧线转向
BASE_EXIT = 10.0             # 回到10°内 → 停止转向(滞回)
BASE_HOLD_S = 0.4            # 持续超阈确认
ARC_FAST = 100               # 弧线外侧轮速 (保持前进!)
ARC_SLOW = 70                # 弧线内侧轮速
ESC_BACK_S = 0.8             # 避障倒车时长(s)
ESC_GIVEUP_S = 3.0           # 阻碍持续多久放弃绕行改等待(s)

obs_state = {"t": 0.0, "blocked": False, "cm": -1}
obs_lock = threading.Lock()

# 主人世界方位角 (云台解耦后的真实偏差, 供follow转向) — [云台]线提供
owner_bearing = {"theta": None}
owner_bearing_lock = threading.Lock()

def _get_owner_bearing():
    with owner_bearing_lock:
        return owner_bearing["theta"]

def _get_yaw_rate():
    """读 /dev/shm/imu.json 的 yaw_rate_dps (imu.service产出)。
    数据过期(>0.5s)或质量差 → None (follow退回开环, 安全网)"""
    try:
        with open("/dev/shm/imu.json") as f:
            d = json.load(f)
        if time.monotonic() - float(d.get("ts", 0)) > 0.5:
            return None
        if float(d.get("quality", 1)) < 0.5:
            return None
        return float(d["yaw_rate_dps"])
    except Exception:
        return None

# ====== 转身到角度 (IMU闭环, 供 [HRI] 动作层) ======
turn_state = {"active": False, "target": None, "yaw": None, "err": None,
              "started": 0.0, "result": "idle"}
turn_lock = threading.Lock()
TURN_TOL_DEG = 3.0      # 到位判定
TURN_TIMEOUT_S = 10.0   # 超时保护
TURN_DIFF = 40          # 转向差速量

def _read_yaw():
    """读积分航向 yaw_deg (逆时针为正); 过期/无效返回 None"""
    try:
        with open("/dev/shm/imu.json") as f:
            d = json.load(f)
        if time.monotonic() - float(d.get("ts", 0)) > 0.5:
            return None
        if float(d.get("quality", 1)) < 0.5:
            return None
        return float(d["yaw_deg"])
    except Exception:
        return None

def _turn_worker(deg, spd):
    y0 = _read_yaw()
    t0 = time.monotonic()
    with turn_lock:
        turn_state.update(active=True, target=round(y0 + deg, 2), yaw=round(y0, 2),
                          err=round(deg, 2), started=t0, result="running")
    try:
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
        print(f"[turn] 结束: {turn_state['result']} yaw={turn_state['yaw']}")

def _turn_start(deg, spd=110):
    if _get_mode() != "manual":      # follow 模式由 follow 循环掌控底盘
        return False
    if turn_state.get("active"):
        return False
    if _read_yaw() is None:          # IMU 不可用则拒绝(宁缺毋滥)
        return False
    threading.Thread(target=_turn_worker, args=(deg, spd), daemon=True).start()
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

def uart_send(cmd):
    with uart_lock:
        try: os.write(uart_fd, f"{cmd}\r\n".encode()); return True
        except: return False

def gimbal_send(body):
    """body: {"pan":deg} 和/或 {"tilt":deg}, 绝对角度。
    带指令限位(防顶死) + 记录最近指令(供斜坡起步)"""
    cmds = []
    if 'pan' in body:
        p = max(PAN_MIN, min(PAN_MAX, float(body['pan'])))
        _last_pose["pan"] = p
        cmds.append(f"PAN {p:.1f}")
    if 'tilt' in body:
        t = max(TILT_MIN, min(TILT_MAX, float(body['tilt'])))
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
                time.sleep(0.05)
        except Exception:
            time.sleep(0.1)

# ====== 跟人控制 (延迟import, 失败不影响主服务) ======

def _follow_loop():
    global fc
    try:
        from follow_controller import FollowController
        fc = FollowController(uart_send, yaw_rate_fn=_get_yaw_rate)
        if not fc.start():
            # 机主原则: 认主失败拒绝跟随, 模式退回手动(凝视线随之退出)
            print("[follow] 认主失败, 跟随未启动")
            _stop_follow()
            return
        while fc.running:
            fc.tick()
            time.sleep(0.4)
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
    同时解算主人世界方位角供follow转向(云台解耦);
    避障: 超声波BLOCKED → 倒车脱离, 持续受阻停车等待(由drive线程执行)。"""
    global gaze
    import urllib.request
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "state"))
    from gaze import GazeController, u_to_bearing
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
            # 世界方位角 = 光学偏角 + 云台已补偿量
            with owner_bearing_lock:
                if u is not None:
                    owner_bearing["theta"] = u_to_bearing(u) + (gaze.pan - PAN_FORWARD_CMD)
                else:
                    owner_bearing["theta"] = None
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

# ====== HTTP Handler ======

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        if self.path == '/': self._page()
        elif self.path == '/ping': self._json({"uart": uart_fd is not None, "mode": _get_mode()})
        elif self.path == '/mode': self._json({"mode": _get_mode()})
        elif self.path == '/turn/status': self._json(turn_state)
        else: self.send_response(302); self.send_header('Location','/'); self.end_headers()
    def do_POST(self):
        if self.path == '/move':
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
                spd = int(body.get('speed', 110))
                if _turn_start(deg, spd):
                    self._json({"ok": True, "state": turn_state})
                else:
                    self._json({"ok": False, "state": turn_state,
                                "error": "busy / imu unavailable / follow mode"})
                return
            except Exception:
                self.send_response(400); self._text('BAD')
                return
        r = {'/forward':'MOVE F 180','/back':'MOVE B 180','/left':'MOVE L 150','/right':'MOVE R 150','/stop':'STOP'}
        if self.path in r:
            self._text("OK" if uart_send(r[self.path]) else "FAIL")
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
window.onload=function(){{setTimeout(function(){{document.getElementById('vfeed').src='http://192.168.4.1:{vp}/stream';}},500);}};
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
    threading.Thread(target=uart_reader, daemon=True).start()
    httpd = TServer(('0.0.0.0', args.port), Handler)
    try: httpd.serve_forever()
    except KeyboardInterrupt: pass
    finally: httpd.shutdown(); os.close(uart_fd) if uart_fd else None

if __name__ == '__main__': main()
