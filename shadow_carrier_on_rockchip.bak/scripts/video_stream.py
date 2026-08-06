#!/usr/bin/env python3
"""
video_stream.py — 实时摄像头 + NPU YOLO + MJPEG推流
=====================================================
用法:
    python3 video_stream.py                     # 默认端口8080
    python3 video_stream.py --port 9090         # 自定义端口
    python3 video_stream.py --no-yolo           # 纯摄像头推流（不用NPU）
    python3 video_stream.py --fps 5             # 限制帧率

Windows观看: 浏览器打开 http://192.168.137.190:8080/
"""

import cv2
import subprocess
import os
import sys
import time
import threading
import socketserver
from http.server import HTTPServer, BaseHTTPRequestHandler
from datetime import datetime

# ============================================================
# 配置
# ============================================================
CAMERA_ID = 10
CAMERA_WIDTH = 1280          # 降分辨率换帧率，1920太慢
CAMERA_HEIGHT = 720
YOLO_BIN = "/home/kickpi/shopping_car_vision/rknn_model_zoo/examples/yolov8/cpp/build/rknn_yolov8_demo"
YOLO_MODEL = "/home/kickpi/shopping_car_vision/rknn_model_zoo/examples/yolov8/cpp/build/model/yolov8.rknn"
YOLO_WORKDIR = "/home/kickpi/shopping_car_vision/rknn_model_zoo/examples/yolov8/cpp/build"
JPEG_QUALITY = 70

# 全局状态：最新一帧的JPEG数据（带YOLO标注）
latest_frame_jpeg = None
latest_detections = []
frame_lock = threading.Lock()
frame_count = 0
fps = 0.0


# ============================================================
# 摄像头 + YOLO 引擎
# ============================================================
class YoloStreamer:
    def __init__(self, camera_id=CAMERA_ID):
        self.cap = None
        self.camera_id = camera_id
        self.running = False
        self.use_yolo = True
        self.target_fps = 0  # 0 = 不限

    def open_camera(self):
        self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        if not self.cap.isOpened():
            print(f"❌ 无法打开摄像头 /dev/video{self.camera_id}")
            return False
        actual_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print(f"📷 摄像头已打开: {int(actual_w)}x{int(actual_h)} @ /dev/video{self.camera_id}")
        return True

    def run_yolo_on_frame(self, frame):
        """对OpenCV帧运行YOLO，返回标注后的图像"""
        global latest_detections

        temp_in = "/tmp/_stream_temp_in.jpg"
        cv2.imwrite(temp_in, frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])

        try:
            result = subprocess.run(
                [YOLO_BIN, YOLO_MODEL, temp_in],
                cwd=YOLO_WORKDIR,
                capture_output=True, text=True, timeout=10
            )
        except subprocess.TimeoutExpired:
            print("⚠️ YOLO推理超时，跳过此帧")
            return frame  # 返回原图

        # 解析检测结果
        dets = []
        for line in result.stdout.split('\n'):
            if '@' in line and '(' in line:
                try:
                    cls, rest = line.split(' @ (', 1)
                    coords, conf = rest.rsplit(') ', 1)
                    x1, y1, x2, y2 = map(int, coords.split())
                    dets.append({
                        'class': cls.strip(),
                        'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                        'confidence': float(conf)
                    })
                except Exception:
                    pass
        latest_detections = dets

        # 读取标注图
        out_path = os.path.join(YOLO_WORKDIR, "out.png")
        if os.path.exists(out_path):
            annotated = cv2.imread(out_path)
            if annotated is not None:
                # 缩放回原始摄像头分辨率
                annotated = cv2.resize(annotated, (frame.shape[1], frame.shape[0]))
                return annotated

        return frame  # 标注图读取失败则返回原图

    def loop(self):
        """主循环：取帧 → YOLO → 编码JPEG → 更新全局帧"""
        global latest_frame_jpeg, frame_count, fps
        self.running = True
        last_time = time.time()
        frame_times = []

        while self.running:
            loop_start = time.time()

            ret, frame = self.cap.read()
            if not ret:
                print("⚠️ 取帧失败")
                time.sleep(0.1)
                continue

            # YOLO推理
            if self.use_yolo:
                frame = self.run_yolo_on_frame(frame)

            # 编码JPEG
            _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])

            with frame_lock:
                latest_frame_jpeg = jpeg.tobytes()
                frame_count += 1

            # FPS统计
            now = time.time()
            frame_times.append(now)
            frame_times = [t for t in frame_times if t > now - 5]  # 5秒窗口
            if len(frame_times) >= 2:
                fps = len(frame_times) / (frame_times[-1] - frame_times[0])

            # 帧率限制
            if self.target_fps > 0:
                elapsed = time.time() - loop_start
                sleep_time = (1.0 / self.target_fps) - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

    def stop(self):
        self.running = False
        if self.cap:
            self.cap.release()


# ============================================================
# MJPEG HTTP 服务器
# ============================================================
class MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # 关闭HTTP日志，不刷屏

    def do_GET(self):
        global latest_frame_jpeg, latest_detections, fps, frame_count

        if self.path == '/' or self.path == '/stream':
            # === MJPEG视频流 ===
            self.send_response(200)
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=--frame')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'close')
            self.end_headers()

            last_sent = b''
            try:
                while True:
                    with frame_lock:
                        jpeg = latest_frame_jpeg
                    if jpeg and jpeg != last_sent:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(f'Content-Length: {len(jpeg)}\r\n\r\n'.encode())
                        self.wfile.write(jpeg)
                        self.wfile.write(b'\r\n')
                        last_sent = jpeg
                    time.sleep(0.03)  # ~30 FPS max
            except (BrokenPipeError, ConnectionResetError):
                pass  # 客户端断开

        elif self.path == '/api/detections':
            # === JSON检测结果 ===
            import json
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            data = json.dumps({
                'fps': round(fps, 1),
                'frame_count': frame_count,
                'detections': latest_detections
            })
            self.wfile.write(data.encode())

        elif self.path == '/viewer':
            # === 浏览器查看器（带检测框列表） ===
            html = '''<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>ShadowCarrier Live</title>
<style>
  *{margin:0;padding:0;box-sizing:border-box}
  body{background:#111;color:#fff;font-family:monospace;display:flex;height:100vh}
  .video{flex:1;display:flex;align-items:center;justify-content:center}
  .video img{max-width:100%;max-height:100vh}
  .panel{width:320px;background:#1a1a1a;padding:20px;overflow-y:auto}
  .panel h2{color:#0f0;margin-bottom:15px}
  .det{background:#222;padding:10px;margin:8px 0;border-radius:6px;border-left:4px solid #0f0}
  .det.person{border-left-color:#0ff}
  .det .cls{font-size:18px;font-weight:bold}
  .det .conf{color:#0f0}
  .det .pos{color:#888;font-size:12px}
  .fps{color:#ff0;font-size:20px;margin:10px 0}
</style></head><body>
<div class="video"><img src="/stream" id="stream"></div>
<div class="panel">
  <h2>🛡️ ShadowCarrier</h2>
  <div class="fps" id="fps">FPS: --</div>
  <div id="dets">等待检测...</div>
</div>
<script>
  setInterval(async()=>{
    try{
      const r=await fetch('/api/detections');
      const d=await r.json();
      document.getElementById('fps').textContent='FPS: '+d.fps+' | 帧: '+d.frame_count;
      if(d.detections && d.detections.length>0){
        document.getElementById('dets').innerHTML=d.detections.map(det=>
          `<div class="det person"><span class="cls">${det.class}</span>
           <span class="conf">${(det.confidence*100).toFixed(1)}%</span>
           <div class="pos">[${det.x1},${det.y1}] → [${det.x2},${det.y2}]</div></div>`
        ).join('');
      }else{
        document.getElementById('dets').textContent='暂未检测到目标';
      }
    }catch(e){}
  },500);
</script></body></html>'''
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(html.encode())

        else:
            self.send_response(302)
            self.send_header('Location', '/viewer')
            self.end_headers()


class ThreadedHTTPServer(socketserver.ThreadingMixIn, HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


# ============================================================
# 入口
# ============================================================
def main():
    import argparse
    parser = argparse.ArgumentParser(description='ShadowCarrier 实时视频流 + YOLO')
    parser.add_argument('--port', type=int, default=8080, help='HTTP端口 (默认8080)')
    parser.add_argument('--no-yolo', action='store_true', help='纯摄像头推流，不用NPU')
    parser.add_argument('--fps', type=int, default=0, help='限制帧率 (0=不限)')
    parser.add_argument('--camera', type=int, default=CAMERA_ID, help='摄像头编号')
    args = parser.parse_args()

    print("=" * 55)
    print("  🛡️ ShadowCarrier — 实时视频 + NPU YOLO 推流")
    print("=" * 55)

    # 启动摄像头+YOLO线程
    streamer = YoloStreamer(camera_id=args.camera)
    streamer.use_yolo = not args.no_yolo
    streamer.target_fps = args.fps

    if not streamer.open_camera():
        sys.exit(1)

    print(f"🧠 YOLO NPU: {'启用' if streamer.use_yolo else '关闭（纯摄像头）'}")
    print(f"🌐 推流地址: http://192.168.137.190:{args.port}/viewer")
    print(f"📡 API接口:  http://192.168.137.190:{args.port}/api/detections")
    print(f"🎥 纯视频流: http://192.168.137.190:{args.port}/stream")
    print()

    thread = threading.Thread(target=streamer.loop, daemon=True)
    thread.start()

    # 启动HTTP服务器（主线程）
    server = ThreadedHTTPServer(('0.0.0.0', args.port), MJPEGHandler)
    print(f"✅ 服务器已启动，Windows浏览器打开上方地址即可观看\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n⏹️ 正在关闭...")
    finally:
        streamer.stop()
        server.shutdown()


if __name__ == '__main__':
    main()
