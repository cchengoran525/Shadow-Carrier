#!/usr/bin/env python3
"""capture.py - 棋盘格自动采集 (板上运行)
每1.5s抓一帧, 检测到9x6棋盘且与上一成功帧差异明显才保存, 存满N张自动结束。
用法: python3 capture.py [张数=20] [输出目录=/tmp/calib]
操作: 屏幕全屏显示 chessboard_screen.png, 举到摄像头前变换姿势即可。
"""
import sys
import time

import cv2
import numpy as np

N = int(sys.argv[1]) if len(sys.argv) > 1 else 20
OUT = sys.argv[2] if len(sys.argv) > 2 else "/tmp/calib"
PATTERN = (9, 6)  # 内角点

import os
os.makedirs(OUT, exist_ok=True)

cap = cv2.VideoCapture(10, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
if not cap.isOpened():
    print("摄像头打开失败 (尝试 /dev/video10)")
    sys.exit(1)

saved = 0
t_start = time.time()
print(f"开始采集 {N} 张, 举着屏幕在摄像头前缓慢变换姿势...")
print("要点: 远近距离变化 / 屏幕移到画面四角 / 左右上下倾斜(<30度)")

# 九宫格分桶采集: 确保画面各区域都被覆盖, 每桶至少1张
CELL_W, CELL_H = 640 // 3, 480 // 3
bucket_count = {}

while saved < N and time.time() - t_start < 300:
    ok, frame = cap.read()
    if not ok:
        time.sleep(0.3)
        continue
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(
        gray, PATTERN,
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        time.sleep(0.8)
        continue
    cx_m, cy_m = corners.mean(axis=0)[0]
    bucket = (int(cx_m // CELL_W), int(cy_m // CELL_H))
    bucket_count[bucket] = bucket_count.get(bucket, 0) + 1
    if bucket_count[bucket] > 3:   # 每个区域最多收3张, 强制你移动
        time.sleep(0.8)
        continue
    covered = len(bucket_count)
    if saved >= 18 and covered < 9:
        # 覆盖不足时不允许提前收工: 继续等未覆盖区域
        time.sleep(0.8)
        continue
    path = f"{OUT}/img_{saved:02d}.jpg"
    cv2.imwrite(path, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
    saved += 1
    print(f"[{saved}/{N}] 已存 {path}  区域{bucket}(该区已收{bucket_count[bucket]}张)", flush=True)
    time.sleep(0.8)

cap.release()
print(f"\n完成: {saved} 张 -> {OUT}")
if saved < 10:
    print("警告: 数量偏少, 建议重跑并多换姿势(特别是画面四角)")
