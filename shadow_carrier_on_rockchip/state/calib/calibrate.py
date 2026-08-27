#!/usr/bin/env python3
"""calibrate.py - 内参标定 (板上运行, 依赖 capture.py 的输出)
读 /tmp/calib/*.jpg -> 亚像素角点 -> calibrateCamera -> 打印结果并存 params
用法: python3 calibrate.py [图像目录=/tmp/calib]
验收: 重投影误差 < 0.5 px, cx,cy 接近 (320,240)±15
"""
import glob
import os
import sys
import time

import cv2
import numpy as np

DIR = sys.argv[1] if len(sys.argv) > 1 else "/tmp/calib"
PATTERN = (9, 6)

files = sorted(glob.glob(f"{DIR}/*.jpg"))
if len(files) < 10:
    print(f"只有 {len(files)} 张, 至少需要 10 张")
    sys.exit(1)

objp = np.zeros((PATTERN[0] * PATTERN[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2)

objpoints, imgpoints, used = [], [], []
for f in files:
    img = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
    found, corners = cv2.findChessboardCorners(
        img, PATTERN,
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        print(f"跳过(未检出): {os.path.basename(f)}")
        continue
    corners = cv2.cornerSubPix(
        img, corners, (11, 11), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
    objpoints.append(objp)
    imgpoints.append(corners)
    used.append(f)

rms, K, dist, _, _ = cv2.calibrateCamera(
    objpoints, imgpoints, img.shape[::-1], None, None)

fx, fy = K[0, 0], K[1, 1]
cx, cy = K[0, 2], K[1, 2]
k1, k2 = dist[0][:2]

print(f"\n使用 {len(used)}/{len(files)} 张")
print(f"重投影误差 rms = {rms:.3f} px   {'✓' if rms < 0.5 else '✗ 偏大,建议重拍'}")
print(f"fx = {fx:.1f}   fy = {fy:.1f}")
print(f"cx = {cx:.1f}   cy = {cy:.1f}   {'✓' if abs(cx-320)<15 and abs(cy-240)<15 else '✗ 偏离中心过多,检查采集是否覆盖四角'}")
print(f"k1 = {k1:.5f}  k2 = {k2:.5f}")
hfov = 2 * np.degrees(np.arctan(320 / fx))
print(f"水平FOV ≈ {hfov:.1f}° (垂直 ≈ {2*np.degrees(np.arctan(240/fy)):.1f}°)")

out = f"""# state/calib/params.py - 自动生成, 勿手改 (重跑 calibrate.py 覆盖)
FX = {fx:.2f}
FY = {fy:.2f}
CX = {cx:.2f}
CY = {cy:.2f}
K1 = {k1:.6f}
K2 = {k2:.6f}
CALIB_RMS = {rms:.4f}
CALIB_IMAGES = {len(used)}
CALIB_DATE = "{time.strftime('%Y-%m-%d')}"
"""
with open(os.path.join(os.path.dirname(__file__), "params.py"), "w") as f:
    f.write(out)
print("\n已写入 state/calib/params.py")
