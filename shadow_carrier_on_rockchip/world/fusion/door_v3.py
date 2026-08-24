#!/usr/bin/env python3
"""
world_lab v3: 门检测终版 —— 几何先验 + 冻结参数
相对 v2 新增(场景无关的物理约束, 非调参):
  - 门宽高比限制 [0.25, 1.3]
  - 门框底边必须到达画面 60% 高度以下 (门是落地的)
参数全部收进 params.py, 本文件不改数值。
"""
import cv2
import numpy as np
import json
import sys
import os
import time
import params as P


def load_image(path):
    img = cv2.imread(path)
    if img is None:
        raise IOError("cannot read " + path)
    h, w = img.shape[:2]
    s = P.WORK_W / w
    return cv2.resize(img, (P.WORK_W, int(h * s)))


def detect_segments(gray):
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    v = np.median(gray)
    edges = cv2.Canny(blur, max(0, int(v * P.CANNY_LO_K)), min(255, int(v * P.CANNY_HI_K)))
    edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=P.HOUGH_THRESHOLD,
                            minLineLength=int(gray.shape[0] * P.HOUGH_MINLEN_RATIO),
                            maxLineGap=P.HOUGH_MAXGAP)
    segs = []
    if lines is None:
        return edges, segs
    for l in lines[:, 0]:
        x1, y1, x2, y2 = map(int, l)
        ang = float(np.degrees(np.arctan2(y2 - y1, x2 - x1)) % 180)
        segs.append({"x1": x1, "y1": y1, "x2": x2, "y2": y2,
                     "ang": ang, "len": float(np.hypot(x2 - x1, y2 - y1))})
    return edges, segs


def merge_collinear(segs):
    segs = [dict(s) for s in segs]
    changed = True
    while changed:
        changed = False
        out = []
        used = [False] * len(segs)
        for i in range(len(segs)):
            if used[i]:
                continue
            a = segs[i]
            best = None
            for j in range(i + 1, len(segs)):
                if used[j]:
                    continue
                b = segs[j]
                d = min(abs(a["ang"] - b["ang"]), 180 - abs(a["ang"] - b["ang"]))
                if d > P.MERGE_ANG_TOL:
                    continue
                mx, my = (b["x1"] + b["x2"]) / 2, (b["y1"] + b["y2"]) / 2
                dx, dy = a["x2"] - a["x1"], a["y2"] - a["y1"]
                L = max(np.hypot(dx, dy), 1e-6)
                dist = abs(dx * (a["y1"] - my) - (a["x1"] - mx) * dy) / L
                if dist > P.MERGE_DIST_TOL:
                    continue
                gap = min(np.hypot(a["x1"] - b["x1"], a["y1"] - b["y1"]),
                          np.hypot(a["x1"] - b["x2"], a["y1"] - b["y2"]),
                          np.hypot(a["x2"] - b["x1"], a["y2"] - b["y1"]),
                          np.hypot(a["x2"] - b["x2"], a["y2"] - b["y2"]))
                if gap > P.MERGE_GAP_TOL:
                    continue
                if best is None or gap < best[1]:
                    best = (j, gap)
            if best is not None:
                j = best[0]
                b = segs[j]
                pts = [(a["x1"], a["y1"]), (a["x2"], a["y2"]),
                       (b["x1"], b["y1"]), (b["x2"], b["y2"])]
                p0 = min(pts, key=lambda p: (p[0], p[1]))
                p1 = max(pts, key=lambda p: (p[0], p[1]))
                out.append({"x1": p0[0], "y1": p0[1], "x2": p1[0], "y2": p1[1],
                            "ang": a["ang"], "len": float(np.hypot(p1[0] - p0[0], p1[1] - p0[1]))})
                used[i] = used[j] = True
                changed = True
            else:
                out.append(a)
                used[i] = True
        segs = out
    return segs


def classify(segs, img_h):
    vs = []
    for s in segs:
        dev = min(abs(s["ang"] - 90), abs(s["ang"] - 270))
        if dev <= P.V_ANG_TOL and s["len"] >= img_h * P.V_MINLEN_RATIO:
            vs.append(s)
    vs.sort(key=lambda s: -s["len"])
    return vs


def pair_doors(vs, w, h, edges):
    """竖线配对 + 几何先验 + 内部密度校验, 全部用冻结参数"""
    cands = []
    for i in range(len(vs)):
        for j in range(i + 1, len(vs)):
            a, b = vs[i], vs[j]
            xa, xb = (a["x1"] + a["x2"]) / 2, (b["x1"] + b["x2"]) / 2
            dx = abs(xa - xb)
            if dx < w * 0.08 or dx > w * 0.55:
                continue
            ya0, ya1 = sorted((a["y1"], a["y2"]))
            yb0, yb1 = sorted((b["y1"], b["y2"]))
            ov = min(ya1, yb1) - max(ya0, yb0)
            if ov < h * P.V_MINLEN_RATIO * 0.5:
                continue
            top = min(ya0, yb0)
            bot = max(ya1, yb1)
            bw, bh = bot - top and dx or 1, bot - top
            aspect = dx / max(bh, 1)
            # 几何先验: 宽高比 & 落地
            if not (P.DOOR_ASPECT_MIN <= aspect <= P.DOOR_ASPECT_MAX):
                continue
            if bot < h * P.DOOR_BOTTOM_MIN_RATIO:
                continue
            ix0, ix1 = int(min(xa, xb) + dx * 0.15), int(max(xa, xb) - dx * 0.15)
            iy0, iy1 = int(top + bh * 0.15), int(bot - bh * 0.1)
            inner = edges[iy0:iy1, ix0:ix1]
            density = float(inner.mean()) / 255.0 if inner.size else 1.0
            center_bonus = 1.0 - abs((xa + xb) / 2 - w / 2) / (w / 2)
            score = (a["len"] + b["len"]) * (0.5 + 0.5 * center_bonus) * (1.0 - min(density * 2, 0.7))
            if score < P.DOOR_SCORE_MIN or density > P.DOOR_INNER_DENS_MAX:
                continue
            cands.append({"x0": int(min(xa, xb)), "x1": int(max(xa, xb)),
                          "y0": int(top), "y1": int(bot),
                          "aspect": round(float(aspect), 2),
                          "inner_density": round(density, 3),
                          "score": round(float(score), 1)})
    cands.sort(key=lambda c: -c["score"])
    kept = []
    for c in cands:
        ok = True
        for k in kept:
            ix = max(0, min(c["x1"], k["x1"]) - max(c["x0"], k["x0"]))
            iy = max(0, min(c["y1"], k["y1"]) - max(c["y0"], k["y0"]))
            if ix * iy > 0.5 * min((c["x1"] - c["x0"]) * (c["y1"] - c["y0"]),
                                   (k["x1"] - k["x0"]) * (k["y1"] - k["y0"])):
                ok = False
                break
        if ok:
            kept.append(c)
    return kept


def traversability(edges, gray):
    h, w = gray.shape
    sub_e, sub_g = edges[h // 2:, :], gray[h // 2:, :]
    ch, cw = sub_e.shape[0] // P.GRID_ROWS, w // P.GRID_COLS
    grid = []
    for r in range(P.GRID_ROWS):
        row = []
        for c in range(P.GRID_COLS):
            ce = sub_e[r * ch:(r + 1) * ch, c * cw:(c + 1) * cw]
            cg = sub_g[r * ch:(r + 1) * ch, c * cw:(c + 1) * cw]
            density = float(ce.mean()) / 255.0
            var = float(cg.var())
            row.append({"density": round(density, 4), "var": round(var, 0),
                        "free": bool(density < P.FREE_DENSITY_MAX and var > P.FREE_VAR_MIN)})
        grid.append(row)
    return grid


def process(path):
    t0 = time.time()
    img = load_image(path)
    t_load = (time.time() - t0) * 1000
    t0 = time.time()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges, segs = detect_segments(gray)
    merged = merge_collinear(segs)
    vs = classify(merged, gray.shape[0])
    doors = pair_doors(vs, gray.shape[1], gray.shape[0], edges)
    grid = traversability(edges, gray)
    t_pipe = (time.time() - t0) * 1000
    res = {"doors": doors, "grid": grid, "n_merged": len(merged), "n_vlines": len(vs),
           "timing_ms": {"load": round(t_load, 1), "pipeline": round(t_pipe, 1)}}
    base = os.path.splitext(os.path.basename(path))[0]
    out = os.path.join(os.path.dirname(os.path.abspath(path)), base + "_v3.jpg")
    vis = img.copy()
    for k, c in enumerate(res["doors"][:3]):
        col = (0, 255, 0) if k == 0 else (0, 200, 255)
        cv2.rectangle(vis, (c["x0"], c["y0"]), (c["x1"], c["y1"]), col, 2)
        cv2.putText(vis, "door%d s=%.0f" % (k, c["score"]),
                    (c["x0"], max(c["y0"] - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1)
    h, w = vis.shape[:2]
    ch, cw = (h // 2) // P.GRID_ROWS, w // P.GRID_COLS
    for r in range(P.GRID_ROWS):
        for c in range(P.GRID_COLS):
            col = (0, 180, 0) if res["grid"][r][c]["free"] else (0, 0, 180)
            cv2.rectangle(vis, (c * cw, h // 2 + r * ch), ((c + 1) * cw, h // 2 + (r + 1) * ch),
                          col, 1 if res["grid"][r][c]["free"] else 2)
    cv2.imwrite(out, vis)
    return out, res
