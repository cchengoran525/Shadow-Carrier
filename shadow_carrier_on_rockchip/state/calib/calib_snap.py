#!/usr/bin/env python3
"""calib_snap.py - 标定手动拍摄服务 (端口8081)
手机浏览器打开 http://192.168.4.1:8081
点按钮 = 抓一帧并检测9x6棋盘格, 找到才存图并反馈。
自动轮询那张图改为被动等待; 重启后仍从已有张数续编。
"""
import glob
import os
import socketserver
import threading

import cv2
from http.server import HTTPServer, BaseHTTPRequestHandler

SAVE_DIR = "/tmp/calib"
PORT = 8081
PATTERN = (9, 6)

os.makedirs(SAVE_DIR, exist_ok=True)

cap = cv2.VideoCapture(10, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
if not cap.isOpened():
    raise SystemExit("摄像头 /dev/video10 打开失败")

lock = threading.Lock()


def next_index():
    files = glob.glob(f"{SAVE_DIR}/img_*.jpg")
    return len(files)


def snap():
    global lock
    with lock:
        ok, frame = cap.read()
        if not ok:
            return False, "读帧失败", None
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray, PATTERN,
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found:
            return False, "未找到棋盘格(挪一下位置/减倾斜/避反光)", None
        cx_m, cy_m = corners.mean(axis=0)[0]
        idx = next_index()
        path = f"{SAVE_DIR}/img_{idx:02d}.jpg"
        cv2.imwrite(path, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
        return True, f"已存 {path}", (int(cx_m), int(cy_m))


PAGE = """<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>标定拍摄</title>
<style>
body{background:#0d1117;color:#c9d1d9;font-family:-apple-system,sans-serif;
     display:flex;flex-direction:column;align-items:center;padding-top:40px}
h1{font-size:22px} #n{font-size:48px;margin:16px}
button{width:220px;height:220px;border-radius:50%;border:0;background:#238636;
       color:#fff;font-size:26px;font-weight:700}
button:active{transform:scale(.95);filter:brightness(.85)}
#msg{margin-top:24px;font-size:16px;color:#8b949e;min-height:44px;text-align:center}
</style></head><body>
<h1>棋盘格标定拍摄</h1>
<div id="n">-</div>
<button onclick="go()">📸 拍一张</button>
<div id="msg">每换好一个姿势点一次</div>
<script>
function go(){
  fetch('/snap',{method:'POST'}).then(r=>r.json()).then(d=>{
    document.getElementById('n').textContent=d.count;
    document.getElementById('msg').textContent=d.msg;
  }).catch(e=>document.getElementById('msg').textContent='网络错误');
}
fetch('/count').then(r=>r.json()).then(d=>document.getElementById('n').textContent=d.count);
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(PAGE.encode())
        elif self.path == "/count":
            self._json({"count": next_index()})

    def do_POST(self):
        if self.path == "/snap":
            ok, msg, center = snap()
            self._json({"ok": ok, "count": next_index(),
                        "msg": msg + (f" 角心{center}" if center else "")})

    def _json(self, d):
        import json
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps(d, ensure_ascii=False).encode())


class S(socketserver.ThreadingMixIn, HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


print(f"标定拍摄服务: http://0.0.0.0:{PORT}  已存{next_index()}张")
S(("0.0.0.0", PORT), Handler).serve_forever()
