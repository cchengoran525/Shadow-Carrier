#!/usr/bin/env python3
"""
world_lab: OpenCV 能力边界探测（离线，独立运行，不依赖工程任何现有代码）
测试项:
  1. 门检测   Canny -> HoughLinesP -> 垂直线配对 -> 矩形候选打分
  2. 墙地交界 画面下部水平线搜索
  3. 可通行性 底半区网格 边缘密度+纹理方差
  4. 消失点   主导线族交点(简化最小二乘)
用法: python3 offline_door_test.py <图片路径> [更多图片...]
输出: 与图片同目录的 <名>_annotated.jpg + report.json + stdout 报告 + 阶段耗时
"""
import cv2
import numpy as np
import json
import sys
import os
import time

WORK_W = 640
V_LINE_ANGLE_TOL = 10      # 垂直线角度容差(度)
H_LINE_ANGLE_TOL = 8       # 水平线角度容差
DOOR_MIN_HEIGHT_RATIO = 0.30   # 门框竖线最短占画面高比例
DOOR_PAIR_X_TOL = 0.45         # 配对两竖线横向间距上限(画面宽比例)
GRID_ROWS = 3
GRID_COLS = 8


def load_image(path):
    img = cv2.imread(path)
    if img is None:
        raise IOError("cannot read " + path)
    h, w = img.shape[:2]
    s = WORK_W / w
    return cv2.resize(img, (WORK_W, int(h * s)))


def detect_lines(gray):
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=60,
                            minLineLength=int(WORK_W * 0.15), maxLineGap=12)
    segs = []
    if lines is None:
        return edges, segs
    for l in lines[:, 0]:
        x1, y1, x2, y2 = l
        ang = np.degrees(np.arctan2(y2 - y1, x2 - x1)) % 180
        length = float(np.hypot(x2 - x1, y2 - y1))
        segs.append({"x1": int(x1), "y1": int(y1), "x2": int(x2), "y2": int(y2),
                     "ang": float(ang), "len": length})
    return edges, segs


def vertical_segs(segs, img_h):
    out = []
    for s in segs:
        dev = min(abs(s["ang"] - 90), abs(s["ang"] - 270))
        if dev <= V_LINE_ANGLE_TOL and s["len"] >= img_h * DOOR_MIN_HEIGHT_RATIO:
            out.append(s)
    out.sort(key=lambda s: -s["len"])
    return out


def horizontal_segs(segs, y_min):
    out = []
    for s in segs:
        dev = min(s["ang"], abs(s["ang"] - 180))
        if dev <= H_LINE_ANGLE_TOL and max(s["y1"], s["y2"]) >= y_min:
            out.append(s)
    out.sort(key=lambda s: -s["len"])
    return out


def pair_door_candidates(vsegs, w, h):
    cands = []
    for i in range(len(vsegs)):
        for j in range(i + 1, len(vsegs)):
            a, b = vsegs[i], vsegs[j]
            xa, xb = (a["x1"] + a["x2"]) / 2, (b["x1"] + b["x2"]) / 2
            dx = abs(xa - xb)
            if dx < w * 0.08 or dx > w * DOOR_PAIR_X_TOL:
                continue
            ya0, ya1 = sorted((a["y1"], a["y2"]))
            yb0, yb1 = sorted((b["y1"], b["y2"]))
            ov = min(ya1, yb1) - max(ya0, yb0)
            if ov < h * DOOR_MIN_HEIGHT_RATIO * 0.6:
                continue
            top = min(ya0, yb0)
            bot = max(ya1, yb1)
            center_bonus = 1.0 - abs((xa + xb) / 2 - w / 2) / (w / 2)
            score = (a["len"] + b["len"]) * (0.5 + 0.5 * center_bonus)
            cands.append({"x0": int(min(xa, xb)), "x1": int(max(xa, xb)),
                          "y0": int(top), "y1": int(bot),
                          "score": round(float(score), 1)})
    cands.sort(key=lambda c: -c["score"])
    return cands


def wall_floor(hsegs):
    for s in hsegs:
        return {"x1": s["x1"], "y1": s["y1"], "x2": s["x2"], "y2": s["y2"],
                "len_ratio": round(s["len"] / WORK_W, 2)}
    return None


def traversability(edges, gray):
    h, w = gray.shape
    sub_edges = edges[h // 2:, :]
    sub_gray = gray[h // 2:, :]
    ch, cw = sub_edges.shape[0] // GRID_ROWS, w // GRID_COLS
    cells = []
    for r in range(GRID_ROWS):
        row = []
        for c in range(GRID_COLS):
            cell_e = sub_edges[r * ch:(r + 1) * ch, c * cw:(c + 1) * cw]
            cell_g = sub_gray[r * ch:(r + 1) * ch, c * cw:(c + 1) * cw]
            density = float(cell_e.mean()) / 255.0
            var = float(cell_g.var())
            free = density < 0.06 and var > 200
            row.append({"density": round(density, 4), "var": round(var, 0),
                        "free": bool(free)})
        cells.append(row)
    return cells


def intersect(a, b):
    d1 = (a["x2"] - a["x1"], a["y2"] - a["y1"])
    d2 = (b["x2"] - b["x1"], b["y2"] - b["y1"])
    den = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(den) < 1e-6:
        return None
    t = ((b["x1"] - a["x1"]) * d2[1] - (b["y1"] - a["y1"]) * d2[0]) / den
    return (a["x1"] + t * d1[0], a["y1"] + t * d1[1])


def vanishing_point(segs, w, h):
    pts = []
    long_segs = sorted(segs, key=lambda s: -s["len"])[:15]
    for i in range(len(long_segs)):
        for j in range(i + 1, len(long_segs)):
            p = intersect(long_segs[i], long_segs[j])
            if p and -w < p[0] < 2 * w and -h < p[1] < 2 * h:
                pts.append(p)
    if len(pts) < 3:
        return None
    arr = np.array(pts)
    cx, cy = float(arr[:, 0].mean()), float(arr[:, 1].mean())
    spread = float(np.hypot(arr[:, 0] - cx, arr[:, 1] - cy).mean())
    return {"x": int(cx), "y": int(cy), "n_inter": len(pts), "spread": round(spread, 1)}


def annotate(img, res):
    vis = img.copy()
    for c in res["doors"][:3]:
        col = (0, 255, 0)
        cv2.rectangle(vis, (c["x0"], c["y0"]), (c["x1"], c["y1"]), col, 2)
        cv2.putText(vis, "door? %.0f" % c["score"], (c["x0"], max(c["y0"] - 6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
    wf = res["wall_floor"]
    if wf:
        cv2.line(vis, (wf["x1"], wf["y1"]), (wf["x2"], wf["y2"]), (255, 120, 0), 3)
    vp = res["vanishing_point"]
    if vp:
        cv2.circle(vis, (vp["x"], vp["y"]), 8, (0, 0, 255), 2)
        cv2.putText(vis, "VP", (vp["x"] + 10, vp["y"]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    h, w = vis.shape[:2]
    ch, cw = (h // 2) // GRID_ROWS, w // GRID_COLS
    for r in range(GRID_ROWS):
        for c in range(GRID_COLS):
            cell = res["grid"][r][c]
            col = (0, 180, 0) if cell["free"] else (0, 0, 180)
            thick = 1 if cell["free"] else 2
            cv2.rectangle(vis, (c * cw, h // 2 + r * ch),
                          ((c + 1) * cw, h // 2 + (r + 1) * ch), col, thick)
    return vis


def process(path):
    t0 = time.time()
    img = load_image(path)
    t_load = time.time() - t0

    t0 = time.time()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges, segs = detect_lines(gray)
    vsegs = vertical_segs(segs, gray.shape[0])
    hsegs = horizontal_segs(segs, gray.shape[0] // 2)
    doors = pair_door_candidates(vsegs, gray.shape[1], gray.shape[0])
    wf = wall_floor(hsegs)
    grid = traversability(edges, gray)
    vp = vanishing_point(segs, gray.shape[1], gray.shape[0])
    t_pipe = time.time() - t0

    res = {
        "doors": doors,
        "wall_floor": wf,
        "vanishing_point": vp,
        "grid": grid,
        "n_vlines": len(vsegs),
        "timing_ms": {"load": round(t_load * 1000, 1), "pipeline": round(t_pipe * 1000, 1)},
    }

    base = os.path.splitext(os.path.basename(path))[0]
    out = os.path.join(os.path.dirname(os.path.abspath(path)), base + "_annotated.jpg")
    cv2.imwrite(out, annotate(img, res))
    return out, res


def main():
    paths = sys.argv[1:]
    if not paths:
        print(__doc__)
        sys.exit(1)
    report = {}
    for p in paths:
        try:
            out, res = process(p)
            report[p] = res
            print("\n=== %s ===" % p)
            print(json.dumps(res, ensure_ascii=False, indent=1))
            print("annotated -> %s" % out)
        except Exception as e:
            print("FAIL %s: %s" % (p, e))
    with open(os.path.join(os.path.dirname(os.path.abspath(paths[0])), "report.json"), "w") as f:
        json.dump(report, f, ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
