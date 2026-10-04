#!/usr/bin/env python3
"""
validate_desk.py —— 桌面尺度极坐标映射验证（棋盘格真值）
原理: 棋盘角点物理间距已知(22mm×格数), 映射后的角点间距应与真值一致。
     棋盘相对相机的绝对位置不需要已知(成对距离对平移不变)。
用法: python3 validate_desk.py
输出: 误差统计 + corners.jpg(角点) + topdown.jpg(俯视: 绿=映射 红=真值)
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
H_DESK = 0.110          # 相机到桌面高度(m), 用户实测
SQUARE_MM = 22.0        # 棋盘格实际边长(量实物)

PATTERNS = [(9, 6), (6, 9), (8, 6), (6, 8), (7, 5), (5, 7), (9, 7), (7, 9), (10, 7), (7, 10)]


def grab_mjpeg_frame(timeout=8.0):
    req = urllib.request.Request(BASE + "/")
    with urllib.request.urlopen(req, timeout=timeout) as r:
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
            gray, pat,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ok:
            crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-3)
            corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), crit)
            return pat, corners[:, 0, :]
    return None, None


def main():
    frame = grab_mjpeg_frame()
    fh, fw = frame.shape[:2]
    print("frame: %dx%d" % (fw, fh))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    pat, pts = find_board(gray)
    if pat is None:
        print("FAIL: 未检出棋盘角点 — 检查板平整/光照/图案")
        cv2.imwrite(os.path.join(OUT, "corners.jpg"), frame)
        sys.exit(1)
    print("棋盘: %dx%d 内角点, 共 %d 个" % (pat[0], pat[1], len(pts)))

    # 1) 映射角点 (pts 顺序 = 行优先: 第r行第c列 => k=r*W+c)
    rows = []
    for k, (u, v) in enumerate(pts):
        r, c = k // pat[0], k % pat[0]
        b, d = pm.pixel_to_polar(u, v, TILT_CMD, PAN_CMD, h_cam=H_DESK)
        rows.append({"r": r, "c": c, "uv": (u, v), "polar": (b, d)})
    rejected = sum(1 for x in rows if x["polar"][0] is None)
    print("拒判(不落地): %d / %d" % (rejected, len(rows)))
    valid = [x for x in rows if x["polar"][0] is not None]

    # 2) 成对距离对比
    errs = []
    for i in range(len(valid)):
        a = valid[i]
        xa = a["polar"][1] * math.cos(math.radians(a["polar"][0]))
        ya = a["polar"][1] * math.sin(math.radians(a["polar"][0]))
        for j in range(i + 1, len(valid)):
            b_ = valid[j]
            xb = b_["polar"][1] * math.cos(math.radians(b_["polar"][0]))
            yb = b_["polar"][1] * math.sin(math.radians(b_["polar"][0]))
            d_map = math.hypot(xa - xb, ya - yb)
            d_true = math.hypot(a["r"] - b_["r"], a["c"] - b_["c"]) * SQUARE_MM / 1000.0
            errs.append((abs(d_map - d_true) / d_true * 100, d_true, d_map))
    errs.sort(key=lambda x: x[0])
    mean_e = sum(e for e, _, _ in errs) / len(errs)
    med_e = errs[len(errs) // 2][0]
    print("\n成对距离误差 (%d 对):" % len(errs))
    print("  中位: %.2f%%   均值: %.2f%%   P90: %.2f%%   最差: %.2f%%"
          % (med_e, mean_e, errs[int(len(errs) * 0.9)][0], errs[-1][0]))
    for lo, hi, name in ((0, 0.3, "<0.3m"), (0.3, 0.5, "0.3-0.5m"), (0.5, 9, "0.5m+")):
        seg = sorted(e for e, dt, _ in errs if lo <= dt < hi)
        if seg:
            print("  真值距离 %s: 中位 %.2f%% (%d 对)" % (name, seg[len(seg) // 2], len(seg)))

    # 3) 俯视对比图: 真值形状(红) vs 映射形状(绿), 质心平移对齐
    scale = 400.0
    arr = np.array([(x["polar"][1] * math.cos(math.radians(x["polar"][0])),
                     x["polar"][1] * math.sin(math.radians(x["polar"][0]))) for x in valid])
    true_xy = np.array([(x["c"] * SQUARE_MM / 1000.0, -x["r"] * SQUARE_MM / 1000.0) for x in valid])
    off = (true_xy - arr).mean(axis=0)

    def P(xm, ym):
        return (int(350 + xm * scale), int(350 + ym * scale))

    top = np.full((700, 700, 3), 255, np.uint8)
    cv2.line(top, (350, 30), (350, 670), (210, 210, 210), 1)
    cv2.line(top, (30, 350), (670, 350), (210, 210, 210), 1)
    cv2.putText(top, "FRONT", (310, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    for (mx, my) in arr:
        cv2.circle(top, P(mx + off[0], my + off[1]), 3, (0, 160, 0), -1)
    for (tx, ty) in true_xy:
        cv2.circle(top, P(tx, ty), 2, (0, 0, 255), -1)
    cv2.putText(top, "green=mapped  red=truth", (10, 685),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    cv2.imwrite(os.path.join(OUT, "topdown.jpg"), top)

    vis = frame.copy()
    cv2.drawChessboardCorners(vis, pat, pts.reshape(-1, 1, 2), True)
    cv2.imwrite(os.path.join(OUT, "corners.jpg"), vis)

    result = {"pattern": list(pat), "n_corners": len(pts), "rejected": rejected,
              "tilt_cmd": TILT_CMD, "pan_cmd": PAN_CMD, "h_desk_m": H_DESK,
              "square_mm": SQUARE_MM,
              "pair_err_median_pct": round(med_e, 2), "pair_err_mean_pct": round(mean_e, 2),
              "pair_err_p90_pct": round(errs[int(len(errs) * 0.9)][0], 2),
              "pair_err_max_pct": round(errs[-1][0], 2)}
    with open(os.path.join(OUT, "desk_validation.json"), "w") as f:
        json.dump(result, f, indent=1)
    print("saved: corners.jpg topdown.jpg desk_validation.json")


if __name__ == "__main__":
    main()
