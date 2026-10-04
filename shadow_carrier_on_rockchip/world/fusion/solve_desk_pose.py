#!/usr/bin/env python3
"""
solve_desk_pose.py —— 用棋盘 PnP 反解相机真实高度/俯角（不量尺）
输入: 一帧 MJPEG + 冻结内参 + 棋盘物理网格(22mm)
输出: 相机到桌面真实高度、光轴真实俯角、棋盘yaw、重投影RMS、
      用解出真值重跑映射后的成对距离误差
"""
import cv2
import numpy as np
import math
import json
import time
import urllib.request
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import polar_mapper as pm

BASE = "http://127.0.0.1:8080"
OUT = os.path.dirname(os.path.abspath(__file__))

TILT_CMD = 130.0
PAN_CMD = 90.0
SQUARE_M = 0.022
PATTERNS = [(9, 6), (6, 9), (8, 6), (6, 8), (7, 5), (5, 7), (9, 7), (7, 9), (10, 7), (7, 10)]

# 冻结内参 (state/calib/params.py)
K = np.array([[pm.FX, 0, pm.CX], [0, pm.FY, pm.CY], [0, 0, 1]], np.float64)
DIST = np.array([pm.K1 if hasattr(pm, "K1") else -0.295070,
                 0.085150, 0, 0], np.float64)  # k1,k2,p1,p2


def grab_mjpeg_frame(timeout=8.0):
    with urllib.request.urlopen(urllib.request.Request(BASE + "/"), timeout=timeout) as r:
        buf = b""
        t0 = time.time()
        while time.time() - t0 < timeout:
            chunk = r.read(4096)
            if not chunk:
                break
            buf += chunk
            e = buf.find(b"\xff\xd9")
            if e != -1:
                s = buf.find(b"\xff\xd8")
                if s != -1 and s < e:
                    img = cv2.imdecode(np.frombuffer(buf[s:e + 2], np.uint8), cv2.IMREAD_COLOR)
                    if img is not None:
                        return img
                    buf = buf[e + 2:]
    raise IOError("no frame")


def find_board(gray):
    for pat in PATTERNS:
        ok, corners = cv2.findChessboardCorners(
            gray, pat, flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ok:
            crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-3)
            corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), crit)
            return pat, corners
    return None, None


def main():
    frame = grab_mjpeg_frame()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    pat, corners = find_board(gray)
    if pat is None:
        print("FAIL: no chessboard")
        sys.exit(1)
    W, H_ = pat
    # 3D 点: 桌面平面 z=0, x沿板行方向, y沿板列方向 (米)
    objp = np.zeros((W * H_, 3), np.float32)
    for k in range(W * H_):
        objp[k, 0] = (k % W) * SQUARE_M
        objp[k, 1] = -(k // W) * SQUARE_M
    ok, rvec, tvec = cv2.solvePnP(objp, corners, K, DIST, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        print("FAIL: solvePnP")
        sys.exit(1)
    proj, _ = cv2.projectPoints(objp, rvec, tvec, K, DIST)
    rms = float(np.sqrt(((proj[:, 0, :] - corners[:, 0, :]) ** 2).sum(axis=1)).mean())
    R, _ = cv2.Rodrigues(rvec)
    # 相机在桌面坐标系中的位置: tvec = 桌面系下相机位置? (PnP: 3D->2D, tvec = 桌面原点在相机系)
    # 反解: 相机中心在桌面系 C = -R^T * tvec
    C = -R.T @ tvec[:, 0]
    h_true = float(C[2])                 # 桌面z=0, 相机z即高度
    # 光轴方向在桌面系: R 的第三行(相机z轴在桌面系) = 光轴向前
    z_axis = R[:, 2]                     # 相机系z(向前)在桌面系坐标
    tilt_true = math.degrees(math.asin(max(-1, min(1, -z_axis[2]))))  # 光轴与水平的夹角(正=下俯)
    yaw_board = math.degrees(math.atan2(R[1, 0], R[0, 0]))

    print("=== PnP 反解 (棋盘%d点, 重投影RMS=%.2fpx) ===" % (W * H_, rms))
    print("相机光心到桌面高度: %.1f mm  (尺量值 110mm)" % (h_true * 1000))
    print("光轴真实俯角: %.2f°  (指令推算 %.1f°)" % (tilt_true, (TILT_CMD - pm.TILT_LEVEL_CMD) * pm.DEG_PER_CMD))
    print("等效 DEG_PER_CMD: %.3f (若 TILT_LEVEL_CMD=112 准确)" %
          ((tilt_true + (TILT_CMD - 112.0)) / (TILT_CMD - 112.0) * pm.DEG_PER_CMD))
    print("棋盘yaw: %.1f°  相机桌面系位置: x=%.3f y=%.3f m" % (yaw_board, C[0], C[1]))

    # 用真值重跑映射: 修正 tilt_down 与 h
    pts2 = corners.reshape(-1, 2)
    errs = []
    mapped = []
    rows = []
    for k, (u, v) in enumerate(pts2):
        r_, c_ = k // W, k % W
        b_, d_ = pm.pixel_to_polar(u, v, tilt_cmd=pm.TILT_LEVEL_CMD + tilt_true,
                                   pan_cmd=PAN_CMD, h_cam=h_true)
        if b_ is not None:
            x = d_ * math.cos(math.radians(b_))
            y = d_ * math.sin(math.radians(b_))
            mapped.append((x, y))
            rows.append((r_, c_))
    for i in range(len(mapped)):
        for j in range(i + 1, len(mapped)):
            dm = math.hypot(mapped[i][0] - mapped[j][0], mapped[i][1] - mapped[j][1])
            dt = math.hypot(rows[i][0] - rows[j][0], rows[i][1] - rows[j][1]) * SQUARE_M
            errs.append(abs(dm - dt) / dt * 100)
    errs.sort()
    print("\n=== 用解出真值后的成对距离误差 (%d 对) ===" % len(errs))
    print("中位: %.2f%%  均值: %.2f%%  P90: %.2f%%  最差: %.2f%%"
          % (errs[len(errs) // 2], sum(errs) / len(errs),
             errs[int(len(errs) * 0.9)], errs[-1]))

    result = {"h_true_mm": round(h_true * 1000, 1), "tilt_true_deg": round(tilt_true, 2),
              "rms_px": round(rms, 2), "yaw_board": round(yaw_board, 1),
              "pair_err_median_pct": round(errs[len(errs) // 2], 2),
              "pair_err_p90_pct": round(errs[int(len(errs) * 0.9)], 2)}
    with open(os.path.join(OUT, "desk_pose_solved.json"), "w") as f:
        json.dump(result, f, indent=1)
    print("saved: desk_pose_solved.json")


if __name__ == "__main__":
    main()
