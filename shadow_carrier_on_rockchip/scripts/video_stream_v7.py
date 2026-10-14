#!/usr/bin/env python3
"""video_stream_v7.py - 背压流水线 + 独立watchdog线程 (v7.3)
v7.1 -> v7.2 改动 (修复select拖慢管线):
  v7.1: consumer用select轮询stderr检测心跳, 端到端降到5fps(select开销)
  v7.2:
    [consumer] 恢复阻塞readline (高效, daemon死时EOF返回)
    [watchdog] 独立线程, 每1s检查daemon退出(poll) + 心跳超时(5s无JSON)
    [背压] Condition计数器, watchdog重启时重置inflight唤醒producer
v7.2 -> v7.3 改动 (修复"USB相机间歇掉线 → 整条管线死 + watchdog空转重启"):
  真根因: USB相机(0c45:6366)间歇 re-enumerate (dmesg: usb 7-1 disconnect/reconnect,
          节点 video10/11/12 漂移)。旧 VideoCapture 持有已删除的 /dev/videoN →
          cap.read() 永久阻塞 → producer 不再投喂 → daemon 无输入而静默 →
          consumer 阻塞在 stderr readline → 心跳超时 → watchdog 每~6s 无脑重启
          daemon (restarts 暴涨) 且不回收子进程 (<defunct> 僵尸); 检测永久失效。
  v7.3:
    [camera]  新增 camera_watchdog 线程: 无新帧 > CAMERA_REOPEN_AFTER 即
              release+reopen 相机(按设备路径重解析, 兼容节点漂移)并清空背压计数;
              相机掉线自动重连, 不再拖垮整条管线
    [producer] 读帧失败/异常不再死循环, 交给 camera_watchdog 统一重连
    [watchdog] 心跳超时仅在"输入仍在流动"时重启 daemon; 相机停摆时不空转重启
    [reap]     daemon 重启前 wait() 回收, 消除 <defunct> 僵尸
    [log]      daemon stderr 的非JSON行/退出码落盘 /dev/shm/yolo_daemon.log
    [stats]    /api/detections 增量暴露 cam_reopens/cam_reads/cam_read_fail/daemon_exits
"""
import cv2, subprocess, os, time, threading, json, socketserver
from http.server import HTTPServer, BaseHTTPRequestHandler

CAMERA_ID = 10
WIDTH, HEIGHT = 640, 480
YOLO_DAEMON = "/home/kickpi/shadow_carrier_on_rockchip/perception/yolo_daemon"
MODEL = "/home/kickpi/shopping_car_vision/rknn_model_zoo/examples/yolov8/cpp/build/model/yolov8.rknn"
YOLO_CWD = "/home/kickpi/shopping_car_vision/rknn_model_zoo/examples/yolov8/cpp/build"
FRAME_A = "/dev/shm/yolo_frame_a.jpg"
FRAME_B = "/dev/shm/yolo_frame_b.jpg"
OUT_IMAGE = "/dev/shm/yolo_out.jpg"
JPEG_QUALITY = 50
MAX_INFLIGHT = 2
HEARTBEAT_TIMEOUT = 5.0
CAMERA_REOPEN_AFTER = 2.5   # s: 超过此时间无成功读帧 → 判定相机异常并重连
DAEMON_LOG = "/dev/shm/yolo_daemon.log"
DAEMON_LOG_MAX = 512 * 1024

latest_jpeg = None
latest_dets = []
fps = 0.0
frame_count = 0
lock = threading.Lock()
daemon = None
daemon_lock = threading.Lock()
stop_flag = False

# 相机 (camera_watchdog 与 producer 共享)
cap = None
cap_lock = threading.Lock()
last_frame_t = time.time()   # 最近一次成功读帧时刻
last_input_t = 0.0           # 最近一次向 daemon 投喂帧路径时刻
log_fh = None

# A3.5: 帧时间戳透传(增量, 不影响既有字段)
latest_ts = 0.0          # epoch ms (daemon 采集代理)
latest_t_mono = 0        # 单调 ms (daemon)
latest_recv_mono = 0     # 本进程收到该帧时的单调 ms (用于算 age)

inflight = 0
inflight_cond = threading.Condition()
last_json_t = time.time()  # 心跳: consumer更新, watchdog检查
stats = {"read_fail": 0, "out_missing": 0, "max_gap": 0.0, "last_gap": 0.0,
         "infer_ms": 0, "restarts": 0,
         "cam_reopens": 0, "cam_reads": 0, "cam_read_fail": 0, "daemon_exits": 0}
stats_lock = threading.Lock()


def _log_daemon_line(text):
    """把 daemon stderr 的 非JSON/诊断行 落盘(带简单轮转), 便于事后查崩溃原因"""
    global log_fh
    try:
        if log_fh is None:
            log_fh = open(DAEMON_LOG, "a", buffering=1)
        log_fh.write(text.rstrip("\n") + "\n")
        if log_fh.tell() > DAEMON_LOG_MAX:
            log_fh.close()
            try:
                os.replace(DAEMON_LOG, DAEMON_LOG + ".1")
            except Exception:
                pass
            log_fh = open(DAEMON_LOG, "a", buffering=1)
    except Exception:
        pass


def _open_camera(cam):
    """按设备路径(推荐 /dev/camera0)打开相机, 重连时会重新解析符号链接"""
    c = cv2.VideoCapture(cam, cv2.CAP_V4L2)
    c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    c.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    c.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    c.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return c


def start_daemon():
    """启动或重启daemon, 回收旧进程(防僵尸), 重置背压计数"""
    global daemon, inflight
    with daemon_lock:
        if daemon is not None:
            try:
                if daemon.poll() is None:
                    daemon.kill()
            except Exception:
                pass
            try:
                rc = daemon.wait(timeout=2)
                _log_daemon_line(f"[watchdog] previous daemon reaped rc={rc}")
            except Exception:
                pass
            for attr in ('stdin', 'stderr'):
                try:
                    fd = getattr(daemon, attr)
                    if fd:
                        fd.close()
                except Exception:
                    pass
        print("[watchdog] 启动yolo_daemon...", flush=True)
        daemon = subprocess.Popen(
            [YOLO_DAEMON, MODEL], cwd=YOLO_CWD,
            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE, text=True, bufsize=1
        )
        _log_daemon_line(f"[watchdog] start daemon pid={daemon.pid}")
    with inflight_cond:
        inflight = 0
        inflight_cond.notify_all()
    with stats_lock:
        stats["restarts"] += 1
    return daemon


def daemon_alive():
    return daemon is not None and daemon.poll() is None


def watchdog_thread():
    """独立watchdog: 检测daemon退出 + 心跳超时(且输入在流动), 自动重启"""
    global last_json_t
    while not stop_flag:
        try:
            alive = daemon_alive()
        except Exception:
            alive = False
        now = time.time()
        if not alive:
            print("[watchdog] daemon退出, 重启", flush=True)
            _log_daemon_line("[watchdog] daemon exited -> restart")
            with stats_lock:
                stats["daemon_exits"] += 1
            start_daemon()
            time.sleep(1.0)
            last_json_t = time.time()
        elif now - last_json_t > HEARTBEAT_TIMEOUT:
            if now - last_input_t < HEARTBEAT_TIMEOUT:
                # 输入在流动却长时间无结果 → daemon 卡死, 重启
                print(f"[watchdog] {now-last_json_t:.0f}s无结果(输入正常), 强制重启", flush=True)
                _log_daemon_line("[watchdog] heartbeat timeout with input flowing -> restart")
                start_daemon()
                time.sleep(1.0)
                last_json_t = time.time()
            else:
                # 无输入(相机停摆): daemon 静默属正常, 不空转重启(交给 camera_watchdog)
                last_json_t = now
        time.sleep(1.0)


def camera_watchdog(cam_id):
    """相机看门狗: 无新帧 > CAMERA_REOPEN_AFTER 即重连(release+reopen, 兼容节点漂移)"""
    global cap, last_frame_t, inflight
    while not stop_flag:
        time.sleep(1.0)
        stalled = time.time() - last_frame_t
        if stalled <= CAMERA_REOPEN_AFTER:
            continue
        print(f"[cam-watchdog] {stalled:.1f}s 无新帧, 重连相机 {cam_id}...", flush=True)
        with stats_lock:
            stats["cam_reopens"] += 1
        with cap_lock:
            old = cap
            try:
                if old is not None:
                    old.release()   # 关闭 fd, 唤醒可能阻塞在 read() 的 producer
            except Exception:
                pass
            try:
                cap = _open_camera(cam_id)
            except Exception:
                cap = None
        # 相机重连 → 之前 in-flight 的帧全部作废, 清空背压计数唤醒 producer
        with inflight_cond:
            inflight = 0
            inflight_cond.notify_all()
        last_frame_t = time.time()   # 给新相机一个宽限期
        time.sleep(1.0)


def producer(cam_id):
    """抓帧线程: 背压控制 + 相机掉线容错(重连交给 camera_watchdog)"""
    global stop_flag, daemon, inflight, cap, last_frame_t, last_input_t
    with cap_lock:
        cap = _open_camera(cam_id)
    if not cap.isOpened():
        print(f"FATAL: cam {cam_id}")
        stop_flag = True
        return
    print(f"Cam: {int(cap.get(3))}x{int(cap.get(4))} ({cam_id})")
    last_frame_t = time.time()

    buf_idx = 0
    while not stop_flag:
        with inflight_cond:
            while inflight >= MAX_INFLIGHT and not stop_flag:
                inflight_cond.wait(timeout=0.5)
            if stop_flag:
                return
            inflight += 1
        with cap_lock:
            c = cap
        try:
            ret, frame = c.read() if c is not None else (False, None)
        except Exception:
            ret, frame = False, None
        if not ret or frame is None:
            with inflight_cond:
                inflight -= 1
                inflight_cond.notify()
            with stats_lock:
                stats["cam_read_fail"] += 1
            time.sleep(0.05)   # 相机异常 → camera_watchdog 会重连
            continue
        with stats_lock:
            stats["cam_reads"] += 1
        last_frame_t = time.time()
        path = FRAME_A if buf_idx else FRAME_B
        buf_idx ^= 1
        cv2.imwrite(path, frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        try:
            d = daemon
            if d is None or d.stdin is None:
                with inflight_cond:
                    inflight -= 1
                    inflight_cond.notify()
                time.sleep(0.1)
                continue
            d.stdin.write(path + '\n')
            d.stdin.flush()
            last_input_t = time.time()
        except (BrokenPipeError, ValueError, OSError):
            with inflight_cond:
                inflight -= 1
                inflight_cond.notify()
            time.sleep(0.2)


def consumer():
    """结果线程: 阻塞readline收JSON(高效) + 推流 + daemon诊断落盘"""
    global latest_jpeg, latest_dets, fps, frame_count, daemon, inflight, last_json_t
    global latest_ts, latest_t_mono, latest_recv_mono
    ftimes = []
    last_t = time.time()
    while not stop_flag:
        try:
            line = daemon.stderr.readline()
        except (ValueError, OSError):
            time.sleep(0.2)
            continue
        if not line:
            time.sleep(0.1)  # EOF: daemon死了, watchdog会重启
            continue
        line = line.strip()
        if not line.startswith('{'):
            _log_daemon_line(line)   # [stage]/模型加载/崩溃诊断 → 落盘
            continue
        try:
            result = json.loads(line)
        except Exception:
            _log_daemon_line("BADJSON " + line)
            continue
        with inflight_cond:
            inflight = max(0, inflight - 1)
            inflight_cond.notify()
        now = time.time()
        last_json_t = now
        gap = now - last_t
        last_t = now
        with stats_lock:
            stats["last_gap"] = round(gap * 1000)
            if gap > stats["max_gap"]:
                stats["max_gap"] = round(gap * 1000)
            stats["infer_ms"] = result.get("ms", 0)
        if "error" in result:
            with stats_lock:
                stats["read_fail"] += 1
            continue
        with lock:
            latest_dets = result.get("det", [])
            latest_ts = result.get("ts", 0.0)
            latest_t_mono = result.get("t_mono", 0)
            latest_recv_mono = time.monotonic() * 1000.0
        try:
            with open(OUT_IMAGE, 'rb') as f:
                jpeg = f.read()
            if jpeg:
                with lock:
                    latest_jpeg = jpeg
                    frame_count += 1
            else:
                with stats_lock:
                    stats["out_missing"] += 1
        except Exception:
            with stats_lock:
                stats["out_missing"] += 1
        ftimes.append(now)
        ftimes = [t for t in ftimes if t > now - 5]
        if len(ftimes) >= 2:
            fps = len(ftimes) / (ftimes[-1] - ftimes[0])


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def do_GET(self):
        if self.path in ('/', '/stream'):
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            last = b''
            try:
                while True:
                    with lock: jpeg = latest_jpeg
                    if jpeg and jpeg != last:
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n')
                        self.wfile.write(f'Content-Length: {len(jpeg)}\r\n\r\n'.encode())
                        self.wfile.write(jpeg); self.wfile.write(b'\r\n')
                        last = jpeg
                    time.sleep(0.01)
            except: pass

        elif self.path == '/api/detections':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            with lock, stats_lock:
                age_ms = round(time.monotonic() * 1000.0 - latest_recv_mono, 1) if latest_recv_mono else None
                self.wfile.write(json.dumps({
                    "fps": round(fps,1), "frames": frame_count,
                    "detections": latest_dets,
                    "ts": latest_ts, "t_mono": latest_t_mono, "age_ms": age_ms,
                    "input_age_ms": round((time.time()-last_input_t)*1000.0,1) if last_input_t else None,
                    "daemon_pid": daemon.pid if daemon is not None else None,
                    "stats": dict(stats)
                }).encode())

        elif self.path == '/viewer':
            html = r'''<!DOCTYPE html><html><head><meta charset="utf-8"><title>ShadowCarrier</title>
<style>*{margin:0;padding:0}body{background:#111;color:#fff;font-family:monospace;display:flex;height:100vh}
.v{flex:1;display:flex;align-items:center;justify-content:center}.v img{max-width:100%;max-height:100vh}
.p{width:320px;background:#1a1a1a;padding:20px;overflow-y:auto}h2{color:#0f0}.fps{color:#ff0;font-size:20px}
.d{background:#222;padding:10px;margin:8px 0;border-radius:6px;border-left:4px solid #0ff}
.d .c{font-size:18px;font-weight:bold}.d .n{color:#0f0}.d .o{color:#888;font-size:12px}
</style></head><body><div class="v"><img src="/stream"></div><div class="p">
<h2>ShadowCarrier</h2><div class="fps" id="f">FPS: --</div><div id="x">...</div></div>
<script>setInterval(async()=>{try{const r=await fetch('/api/detections');const d=await r.json();
f.textContent='FPS: '+d.fps+' | '+d.frames+' frames'+(d.stats?' | restarts:'+d.stats.restarts+' cam_reopen:'+d.stats.cam_reopens+' infer:'+d.stats.infer_ms+'ms fail:'+d.stats.read_fail:'');
if(d.detections&&d.detections.length){x.innerHTML=d.detections.map(dd=>
`<div class="d"><span class="c">${dd.c}</span> <span class="n">${(dd.p*100).toFixed(0)}%</span>
<div class="o">[${dd.x1},${dd.y1}]→[${dd.x2},${dd.y2}]</div></div>`).join('');}
else{x.textContent='no detections';}}catch(e){}},500);</script></body></html>'''
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(html.encode())
        else:
            self.send_response(302); self.send_header('Location', '/viewer'); self.end_headers()


class Threaded(socketserver.ThreadingMixIn, HTTPServer):
    allow_reuse_address = True; daemon_threads = True


def main():
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('--port', type=int, default=8080)
    p.add_argument('--camera', type=str, default=str(CAMERA_ID),
                   help="数字索引(如10)或设备路径(如/dev/camera0); USB插拔会让索引漂移,推荐路径")
    args = p.parse_args()

    cam = args.camera
    if isinstance(cam, str) and cam.isdigit():
        cam = int(cam)
    print("Starting yolo_daemon (watchdog v7.3)...")
    start_daemon()
    time.sleep(0.5)
    print(f"Camera {cam}...")
    threading.Thread(target=producer, args=(cam,), daemon=True).start()
    threading.Thread(target=consumer, daemon=True).start()
    threading.Thread(target=watchdog_thread, daemon=True).start()
    threading.Thread(target=camera_watchdog, args=(cam,), daemon=True).start()
    time.sleep(2)
    print(f"Ready: http://192.168.137.190:{args.port}/viewer")
    httpd = Threaded(('0.0.0.0', args.port), Handler)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        global stop_flag
        stop_flag = True
        httpd.shutdown()
        if daemon: daemon.stdin.close(); daemon.terminate()


if __name__ == '__main__':
    main()
