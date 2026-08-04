#!/usr/bin/env python3
"""repro.py - 复现 v3 管线死锁, 记录各阶段耗时定位卡点"""
import cv2, subprocess, time, json, sys

CAMERA_ID = 10
DAEMON = "/home/kickpi/shadow_carrier_on_rockchip/perception/yolo_daemon"
MODEL = "/home/kickpi/shopping_car_vision/rknn_model_zoo/examples/yolov8/cpp/build/model/yolov8.rknn"
YOLO_CWD = "/home/kickpi/shopping_car_vision/rknn_model_zoo/examples/yolov8/cpp/build"
TEMP_IMAGE = "/dev/shm/yolo_frame.jpg"
OUT_IMAGE = "/dev/shm/yolo_out.jpg"

def report(pname, msg):
    print(f"[{pname}] {msg}", flush=True)

cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
if not cap.isOpened():
    print("FATAL: camera"); sys.exit(1)

daemon = subprocess.Popen([DAEMON, MODEL], cwd=YOLO_CWD,
    stdin=subprocess.PIPE, stdout=subprocess.PIPE,
    stderr=subprocess.DEVNULL, text=True, bufsize=1)
time.sleep(1)

frame = 0
t_start = time.time()
t_last_report = time.time()
while time.time() - t_start < 60:
    frame += 1
    # 阶段1: cap.read
    t = time.time()
    ret, img = cap.read()
    t_cap = time.time() - t
    if not ret:
        report("cap", f"frame{frame} cap.read失败")
        continue

    # 阶段2: imwrite
    cv2.imwrite(TEMP_IMAGE, img, [cv2.IMWRITE_JPEG_QUALITY, 75])
    t_write = time.time() - t - t_cap

    # 阶段3: 发路径 + 等JSON
    daemon.stdin.write(TEMP_IMAGE + '\n'); daemon.stdin.flush()
    line = daemon.stdout.readline()
    t_infer = time.time() - t - t_cap - t_write
    if not line:
        report("daemon", f"frame{frame} daemon无响应(EOF?)")
        break

    # 阶段4: 读out.jpg
    try:
        with open(OUT_IMAGE, 'rb') as f:
            jpeg = f.read()
    except Exception as e:
        report("out", f"frame{frame} 读out失败 {e}")
        jpeg = b''
    t_read = time.time() - t - t_cap - t_write - t_infer

    # 心跳报告
    if time.time() - t_last_report > 3:
        t_last_report = time.time()
        total = time.time() - t
        report("heart", f"frame{frame} cap={t_cap*1000:.0f}ms write={t_write*1000:.0f}ms infer={t_infer*1000:.0f}ms read={t_read*1000:.0f}ms jpeg={len(jpeg)//1024}KB resp={line[:60]}")
    if frame % 20 == 0:
        report("prog", f"frame{frame} @{time.time()-t_start:.0f}s")

# 结束: 打印状态
report("end", f"循环结束 frame={frame} 用时{time.time()-t_start:.0f}s")
try:
    cap.release()
    daemon.terminate()
except: pass
