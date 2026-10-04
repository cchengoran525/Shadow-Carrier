#!/usr/bin/env python3
"""
auto_calib_pose.py —— 相机位姿一键重标定（换相机/换位置后跑一遍即可）
流程:
  1. 遍历俯仰指令列表, 每档: /gimbal 下发 → 等舵机到位 → MJPEG 抓帧 →
     棋盘PnP → 记录 (tilt_cmd, h_true, tilt_true)
  2. 线性拟合 h(tilt_cmd) 与 tilt_true(tilt_cmd)
  3. 存 calib_pose.json + calib_report.md, 云台恢复初始位
依赖: 桌面/地面上平铺 9x6 棋盘(22mm格), 环境不动
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

BASE_CTRL = "http://127.0.0.1:80"       # rk_control
BASE_STREAM = "http://127.0.0.1:8080"   # video_stream
OUT = os.path.dirname(os.path.abspath(__file__))

SQUARE_M = 0.022
PATTERNS = [(9, 6), (6, 9), (8, 6), (6, 8), (7, 5), (5, 7), (9, 7), (7, 9), (10, 7), (7, 10)]
TILT_SWEEP = [122, 126, 130, 134, 138]  # 下俯扫掠档位(默认桌面); CLI可覆盖
SETTLE_S = 1.0

K = np.array([[pm.FX, 0, pm.CX], [0, pm.FY, pm.CY], [0, 0, 1]], np.float64)
DIST = np.array([-0.295070, 0.085150, 0, 0], np.float64)


def gimbal(pan=None, tilt=None):
    body = {}
    if pan is not None:
        body["pan"] = pan
    if tilt is not None:
        body["tilt"] = tilt
    req = urllib.request.Request(BASE_CTRL + "/gimbal",
                                 data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as r:
        return r.status == 200


def grab_mjpeg_frame(timeout=8.0):
    with urllib.request.urlopen(urllib.request.Request(BASE_STREAM + "/"), timeout=timeout) as r:
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


def solve_pose(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    pat, corners = find_board(gray)
    if pat is None:
        return None
    Wp, Hp = pat
    objp = np.zeros((Wp * Hp, 3), np.float32)
    for k in range(Wp * Hp):
        objp[k, 0] = (k % Wp) * SQUARE_M
        objp[k, 1] = -(k // Wp) * SQUARE_M
    ok, rvec, tvec = cv2.solvePnP(objp, corners, K, DIST, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    proj, _ = cv2.projectPoints(objp, rvec, tvec, K, DIST)
    rms = float(np.sqrt(((proj[:, 0, :] - corners[:, 0, :]) ** 2).sum(axis=1)).mean())
    R, _ = cv2.Rodrigues(rvec)
    C = -R.T @ tvec[:, 0]
    z_axis = R[:, 2]
    tilt_true = math.degrees(math.asin(max(-1, min(1, -z_axis[2]))))
    return {"h_mm": float(C[2]) * 1000, "tilt_true": tilt_true, "rms": rms,
            "n": Wp * Hp, "pattern": list(pat)}


def fitline(xs, ys):
    A = np.vstack([xs, np.ones(len(xs))]).T
    k, b = np.linalg.lstsq(A, np.array(ys), rcond=None)[0]
    resid = np.array(ys) - (k * np.array(xs) + b)
    return float(k), float(b), float(np.abs(resid).max())


def main():
    global TILT_SWEEP
    if len(sys.argv) > 1 and sys.argv[1] == "sweep":
        TILT_SWEEP = [float(x) for x in sys.argv[2:]]
    print("=== 相机位姿自动标定 ===")
    print("扫掠俯仰档位: %s  (棋盘需平铺在视野内)" % TILT_SWEEP)
    samples = []
    for tilt in TILT_SWEEP:
        if not gimbal(tilt=tilt):
            print("tilt=%d: /gimbal 下发失败, 跳过" % tilt)
            continue
        time.sleep(SETTLE_S)
        try:
            frame = grab_mjpeg_frame()
        except IOError as e:
            print("tilt=%d: 抓帧失败 %s" % (tilt, e))
            continue
        res = solve_pose(frame)
        if res is None:
            print("tilt=%d: 未检出棋盘, 跳过" % tilt)
            continue
        res["tilt_cmd"] = tilt
        samples.append(res)
        print("tilt=%3d: h=%6.1fmm  真实俯角=%6.2f°  RMS=%.2fpx (%d点)"
              % (tilt, res["h_mm"], res["tilt_true"], res["rms"], res["n"]))

    if len(samples) < 3:
        print("\nFAIL: 有效点不足3个, 无法拟合 — 检查棋盘覆盖/光照")
        gimbal(tilt=130)
        sys.exit(1)

    xs = [s["tilt_cmd"] for s in samples]
    hk, hb, hmax = fitline(xs, [s["h_mm"] for s in samples])
    tk, tb, tmax = fitline(xs, [s["tilt_true"] for s in samples])
    print("\n=== 拟合结果 ===")
    print("H(tilt_cmd)  = %.4f * cmd %+.1f   (最大残差 %.1fmm)" % (hk, hb, hmax))
    print("tilt_true(cmd) = %.4f * cmd %+.1f  (最大残差 %.2f°)" % (tk, tb, tmax))
    print("等效 DEG_PER_CMD = %.3f, 零俯角指令 = %.1f" % (tk, -tb / tk))

    out = {"date": time.strftime("%Y-%m-%d %H:%M"),
           "square_m": SQUARE_M,
           "samples": samples,
           "fit_h": {"k": hk, "b": hb, "max_resid_mm": hmax},
           "fit_tilt": {"k": tk, "b": tb, "max_resid_deg": tmax},
           "zero_tilt_cmd": round(-tb / tk, 2)}
    with open(os.path.join(OUT, "calib_pose.json"), "w") as f:
        json.dump(out, f, indent=1)

    md = ["# 相机位姿标定报告 (%s)" % out["date"], "",
          "| tilt指令 | 光心高度mm | 真实俯角° | RMS px | 角点 |", "|---|---|---|---|---|"]
    for s in samples:
        md.append("| %d | %.1f | %.2f | %.2f | %d |" % (s["tilt_cmd"], s["h_mm"], s["tilt_true"], s["rms"], s["n"]))
    md += ["", "H(cmd) = %.4f·cmd %+.1f (残差≤%.1fmm)" % (hk, hb, hmax),
           "tilt_true(cmd) = %.4f·cmd %+.1f (残差≤%.2f°)" % (tk, tb, tmax),
           "等效零俯角指令 = %.1f, DEG_PER_CMD = %.3f" % (-tb / tk, tk), "",
           "> 换相机/换位置后重跑 auto_calib_pose.py 即可刷新本报告"]
    with open(os.path.join(OUT, "calib_report.md"), "w") as f:
        f.write("\n".join(md))

    gimbal(tilt=130)
    print("\nsaved: calib_pose.json calib_report.md  (云台已恢复 tilt=130)")


if __name__ == "__main__":
    main()
