#!/usr/bin/env python3
"""pan_sweep.py - 旋转扫描自标定焦距 (不依赖任何卷尺测量)
原理: 固定场景(棋盘格屏幕), 云台按指令角步进,
      亚像素追踪同一角点, 用 u = cx + fx*tan(s*(cmd-theta0)) 拟合。
输出: fx_rot / cx_refined / 各档残差 / 判读
运行前: 先 POST /gimbal 把云台设为 tilt=112, 之后本脚本只改 pan。
"""
import json
import time
import urllib.request

import cv2
import numpy as np

PAN_LO, PAN_HI, STEP = 40, 120, 10
TILT_LOCK = 112
PATTERN = (9, 6)
SETTLE_S = 0.9

CTRL = "http://127.0.0.1:80"


def post_gimbal(pan):
    req = urllib.request.Request(
        f"{CTRL}/gimbal", method="POST",
        data=json.dumps({"pan": pan}).encode(),
        headers={"Content-Type": "application/json"})
    urllib.request.urlopen(req, timeout=3).read()


def grab_corner():
    cap = None
    # 摄像头被 video-stream 占用时无法双开; 本脚本要求先停 video-stream
    cap = cv2.VideoCapture(10, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    time.sleep(0.4)
    best = None
    for attempt in range(6):          # 每档最多试6帧
        ok, frame = cap.read()
        if not ok:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray, PATTERN,
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if found:
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
            # 取角点阵列的左上第一个角点作为追踪特征
            best = corners[0][0].copy()
            break
    cap.release()
    return best


def main():
    xs, thetas = [], []
    print("旋转扫描开始")
    for pan in range(PAN_LO, PAN_HI + 1, STEP):
        try:
            post_gimbal(pan)
        except Exception as e:
            print(f"pan={pan}: 下发失败 {e}")
            continue
        time.sleep(SETTLE_S)
        pt = grab_corner()
        if pt is None:
            print(f"pan={pan}: 未检出棋盘格(跳过)")
            continue
        xs.append(float(pt[0]))
        thetas.append(float(pan))
        print(f"pan={pan}: 角点u={pt[0]:.1f}")

    if len(xs) < 5:
        print("有效点不足5个, 放弃拟合 (检查画面覆盖/光照)")
        return

    U = np.array(xs)
    C = np.array(thetas)

    def fit(p):
        s, th0, fx, cx = p
        return cx + fx * np.tan(np.radians(s * (C - th0)))

    p0 = np.array([1.0, 80.0, 600.0, 320.0])
    p = p0.copy()
    for _ in range(500):
        pred = fit(p)
        r = U - pred
        J = np.zeros((len(C), 4))
        eps = 1e-5
        for k in range(4):
            q = p.copy()
            q[k] += eps
            J[:, k] = (fit(q) - pred) / eps
        dp, *_ = np.linalg.lstsq(J, r, rcond=None)
        p += dp
        if np.abs(dp).max() < 1e-7:
            break

    s, th0, fx, cx = p
    resid = U - fit(p)
    rms = float(np.sqrt((resid ** 2).mean()))

    print("\n===== 拟合结果 =====")
    print(f"s(方向±1)         = {s:+.3f}")
    print(f"theta0 (u=cx时)   = {th0:.2f} (deg)")
    print(f"fx_rot            = {fx:.1f} px   ← 本次主产物")
    print(f"cx_refined        = {cx:.1f} px")
    print(f"残差 rms          = {rms:.2f} px")
    print("\n各档残差(px):")
    for c, u, rr in zip(C, U, resid):
        print(f"  pan={c:.0f}: u={u:7.1f}  resid={rr:+.2f}")
    print("\n判读:")
    lo, hi = sorted([fx, 0])
    if 480 <= fx <= 700:
        print("  fx_rot 与第1轮(508)/第2轮(606)同区 → 第4轮757确认是退化解")
    elif fx > 700:
        print("  fx_rot 偏大, 与第4轮接近 → 需复查(舵机方向抖动/特征漂移?)")

    out = {"s": float(s), "theta0_deg": float(th0), "fx_rot": float(fx),
           "cx": float(cx), "rms_px": rms,
           "samples": [{"pan": float(c), "u": float(u)} for c, u in zip(C, U)]}
    json.dump(out, open("/tmp/pan_sweep.json", "w"), indent=1)
    print("已存 /tmp/pan_sweep.json")


if __name__ == "__main__":
    main()
