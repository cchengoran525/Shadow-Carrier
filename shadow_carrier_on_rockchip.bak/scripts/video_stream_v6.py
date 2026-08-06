#!/usr/bin/env python3
"""video_stream_v6.py - 背压流水线 (v6)
v5 -> v6 改动 (解决延迟累积 + 花屏/无结果):
  v5 bug: producer无背压, 抓帧速率(75ms) > daemon处理(132ms)
    → 路径积压无限增长 → 延迟越来越大
    → 积压时帧时序错乱 → daemon读到producer正在写的frame → 坏帧
    → read_fail → 检测无结果; 坏帧out → 花屏
  v6 修复:
    [背压] inflight计数(最多2帧在飞), producer等consumer收结果再喂
      → 延迟有界(~180ms), 帧读写时序规整
    [daemon v4] out.jpg tmp+rename原子替换 → consumer永远读到完整帧
    [诊断] API暴露 read_fail计数 / 最大帧间隔, 便于定位异常
"""
import cv2, subprocess, os, time, threading, json, socketserver
from http.server import HTTPServer, BaseHTTPRequestHandler

CAMERA_ID = 10
WIDTH, HEIGHT = 1280, 720
YOLO_DAEMON = "/home/kickpi/shadow_carrier_on_rockchip/perception/yolo_daemon"
MODEL = "/home/kickpi/shopping_car_vision/rknn_model_zoo/examples/yolov8/cpp/build/model/yolov8.rknn"
YOLO_CWD = "/home/kickpi/shopping_car_vision/rknn_model_zoo/examples/yolov8/cpp/build"
FRAME_A = "/dev/shm/yolo_frame_a.jpg"
FRAME_B = "/dev/shm/yolo_frame_b.jpg"
OUT_IMAGE = "/dev/shm/yolo_out.jpg"
JPEG_QUALITY = 75
MAX_INFLIGHT = 2  # 背压: 最多2帧在飞(daemon处理中)

latest_jpeg = None
latest_dets = []
fps = 0.0
frame_count = 0
lock = threading.Lock()
daemon = None
stop_flag = False

# 背压 + 诊断
inflight = 0
inflight_lock = threading.Lock()
stats = {"read_fail": 0, "out_missing": 0, "max_gap": 0.0, "last_gap": 0.0}
stats_lock = threading.Lock()


def start_daemon():
    global daemon
    daemon = subprocess.Popen(
        [YOLO_DAEMON, MODEL], cwd=YOLO_CWD,
        stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE, text=True, bufsize=1
    )


def producer(cam_id):
    """抓帧线程: 背压控制, 最多MAX_INFLIGHT帧在飞"""
    global stop_flag, inflight
    cap = cv2.VideoCapture(cam_id, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    if not cap.isOpened():
        print(f"FATAL: cam /dev/video{cam_id}")
        stop_flag = True
        return
    print(f"Cam: {int(cap.get(3))}x{int(cap.get(4))}")

    buf_idx = 0
    while not stop_flag:
        # 背压: 在飞帧满则等待 (避免路径积压/帧竞争)
        with inflight_lock:
            if inflight >= MAX_INFLIGHT:
                time.sleep(0.004)
                continue
        ret, frame = cap.read()
        if not ret: time.sleep(0.01); continue
        path = FRAME_A if buf_idx else FRAME_B
        buf_idx ^= 1
        cv2.imwrite(path, frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        daemon.stdin.write(path + '\n')
        daemon.stdin.flush()
        with inflight_lock:
            inflight += 1


def consumer():
    """结果线程: 收JSON→读out.jpg→更新latest, 递减inflight"""
    global latest_jpeg, latest_dets, fps, frame_count, inflight
    ftimes = []
    last_t = time.time()
    while not stop_flag:
        line = daemon.stderr.readline()
        if not line:
            time.sleep(0.01); continue
        line = line.strip()
        if not line.startswith('{'):
            continue
        try:
            result = json.loads(line)
        except Exception:
            continue
        # 递减背压计数
        with inflight_lock:
            inflight = max(0, inflight - 1)
        # 诊断: 帧间隔
        now = time.time()
        gap = now - last_t
        last_t = now
        with stats_lock:
            stats["last_gap"] = round(gap * 1000)
            if gap > stats["max_gap"]:
                stats["max_gap"] = round(gap * 1000)
        if "error" in result:
            with stats_lock:
                stats["read_fail"] += 1
            continue
        latest_dets = result.get("det", [])
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
                    time.sleep(0.02)
            except: pass

        elif self.path == '/api/detections':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            with lock, stats_lock:
                self.wfile.write(json.dumps({
                    "fps": round(fps,1), "frames": frame_count,
                    "detections": latest_dets,
                    "stats": dict(stats), "inflight": inflight
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
f.textContent='FPS: '+d.fps+' | '+d.frames+' frames'+(d.stats?' | fail:'+d.stats.read_fail+' gap:'+d.stats.max_gap+'ms':'');
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
    p.add_argument('--camera', type=int, default=CAMERA_ID)
    args = p.parse_args()

    print("Starting yolo_daemon...")
    start_daemon()
    time.sleep(0.5)
    print(f"Camera /dev/video{args.camera}...")
    threading.Thread(target=producer, args=(args.camera,), daemon=True).start()
    threading.Thread(target=consumer, daemon=True).start()
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
