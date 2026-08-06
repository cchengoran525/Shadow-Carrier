#!/usr/bin/env python3
"""video_stream_v5.py - 双线程流水线 (v5)
v4 -> v5 改动 (流水线并行化):
  v4: 单线程串行 (抓帧75ms → 等daemon 130ms → 读out 1ms = 5fps)
  v5: 双线程并行流水线
    [生产者] 抓帧 → imwrite双缓冲 → 发路径给daemon (不等结果)
    [daemon] 连续推理 (77ms/帧 → ~12fps)
    [消费者] 从stderr读JSON → 读out.jpg → 更新latest_jpeg (收结果)
  FPS = max(生产者, daemon) ≈ 10-12fps
双缓冲: frame_a.jpg/frame_b.jpg 交替, daemon读一个时生产者写另一个
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

latest_jpeg = None
latest_dets = []
fps = 0.0
frame_count = 0
lock = threading.Lock()
daemon = None
stop_flag = False


def start_daemon():
    global daemon
    daemon = subprocess.Popen(
        [YOLO_DAEMON, MODEL], cwd=YOLO_CWD,
        stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE, text=True, bufsize=1
    )


def producer(cam_id):
    """抓帧线程: 抓帧→编码→双缓冲写→发路径, 不等待结果"""
    global stop_flag
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
        ret, frame = cap.read()
        if not ret: time.sleep(0.01); continue
        # 双缓冲交替写, daemon读一个时我们写另一个
        path = FRAME_A if buf_idx else FRAME_B
        buf_idx ^= 1
        cv2.imwrite(path, frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        daemon.stdin.write(path + '\n')
        daemon.stdin.flush()


def consumer():
    """结果线程: 从daemon stderr收JSON→读out.jpg→更新latest_jpeg"""
    global latest_jpeg, latest_dets, fps, frame_count
    ftimes = []
    while not stop_flag:
        line = daemon.stderr.readline()
        if not line:
            time.sleep(0.01); continue
        line = line.strip()
        if not line.startswith('{'):
            continue  # 调试日志([stage]等), 忽略
        try:
            result = json.loads(line)
        except Exception:
            continue
        if "error" in result:
            continue
        latest_dets = result.get("det", [])
        try:
            with open(OUT_IMAGE, 'rb') as f:
                jpeg = f.read()
            if jpeg:
                with lock:
                    latest_jpeg = jpeg
                    frame_count += 1
        except Exception:
            pass
        # FPS统计 (结果到达速率 = 真实检测帧率)
        now = time.time()
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
            with lock:
                self.wfile.write(json.dumps({"fps": round(fps,1), "frames": frame_count, "detections": latest_dets}).encode())

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
f.textContent='FPS: '+d.fps+' | '+d.frames+' frames';
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
