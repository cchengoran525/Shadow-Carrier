#!/usr/bin/env python3
"""
world_lab v2: 门检测二轮 —— 针对低对比度门框的改进
改进点:
  1. 自适应 Canny (median ± 60/180) 替代固定 50/150
  2. 膨胀边缘连接断裂线段, 再 Hough
  3. 近共线线段合并 (角度<6度, 端距<40px) 拼回长竖线
  4. 放宽配对: 高度比 0.30->0.22, 角度容差 10->14
  5. 候选加 "内部边缘密度" 校验 (真门框内部相对干净)
用法: python3 offline_door_test_v2.py <img> [...]
"""
import cv2
import numpy as np
import json
import sys
import os
import time

WORK_W = 640
GRID_ROWS = 3
GRID_COLS = 8


def load_image(path):
    img = cv2.imread(path)
    if img is None:
        raise IOError("cannot read " + path)
    h, w = img.shape[:2]
    s = WORK_W / w
    return cv2.resize(img, (WORK_W, int(h * s)))


def auto_canny(gray):
    v = np.median(gray)
    lo = max(0, int(v * 0.55))
    hi = min(255, int(v * 1.45))
    return cv2.Canny(gray, lo, hi)


def detect_segments(gray):
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = auto_canny(blur)
    edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=40,
                            minLineLength=int(gray.shape[0] * 0.18), maxLineGap=25)
    segs = []
    if lines is None:
        return edges, segs
    for l in lines[:, 0]:
        x1, y1, x2, y2 = map(int, l)
        ang = float(np.degrees(np.arctan2(y2 - y1, x2 - x1)) % 180)
        segs.append({"x1": x1, "y1": y1, "x2": x2, "y2": y2,
                     "ang": ang, "len": float(np.hypot(x2 - x1, y2 - y1))})
    return edges, segs


def merge_collinear(segs, ang_tol=6.0, gap_tol=40):
    """把近共线的短段拼成长段(迭代)"""
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
                if d > ang_tol:
                    continue
                # b 的中点到 a 所在直线的距离
                mx, my = (b["x1"] + b["x2"]) / 2, (b["y1"] + b["y2"]) / 2
                dx, dy = a["x2"] - a["x1"], a["y2"] - a["y1"]
                L = max(np.hypot(dx, dy), 1e-6)
                dist = abs(dx * (a["y1"] - my) - (a["x1"] - mx) * dy) / L
                if dist > 8:
                    continue
                # 端点间隙
                gap = min(np.hypot(a["x1"] - b["x1"], a["y1"] - b["y1"]),
                          np.hypot(a["x1"] - b["x2"], a["y1"] - b["y2"]),
                          np.hypot(a["x2"] - b["x1"], a["y2"] - b["y1"]),
                          np.hypot(a["x2"] - b["x2"], a["y2"] - b["y2"]))
                if gap > gap_tol:
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
                m = {"x1": p0[0], "y1": p0[1], "x2": p1[0], "y2": p1[1],
                     "ang": a["ang"], "len": float(np.hypot(p1[0] - p0[0], p1[1] - p0[1]))}
                out.append(m)
                used[i] = used[j] = True
                changed = True
            else:
                out.append(a)
                used[i] = True
        segs = out
    return segs


def classify(segs, img_h):
    vs, hs = [], []
    for s in segs:
        dev_v = min(abs(s["ang"] - 90), abs(s["ang"] - 270))
        dev_h = min(s["ang"], abs(s["ang"] - 180))
        if dev_v <= 14 and s["len"] >= img_h * 0.22:
            vs.append(s)
        elif dev_h <= 8 and s["len"] >= WORK_W * 0.25:
            hs.append(s)
    vs.sort(key=lambda s: -s["len"])
    hs.sort(key=lambda s: -s["len"])
    return vs, hs


def pair_doors(vs, w, h, edges):
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
            if ov < h * 0.22 * 0.5:
                continue
            top = min(ya0, yb0)
            bot = max(ya1, yb1)
            # 内部边缘密度校验: 真门板内部边缘应明显少于门框
            ix0, ix1 = int(min(xa, xb) + dx * 0.15), int(max(xa, xb) - dx * 0.15)
            iy0, iy1 = int(top + (bot - top) * 0.15), int(bot - (bot - top) * 0.1)
            inner = edges[iy0:iy1, ix0:ix1]
            density = float(inner.mean()) / 255.0 if inner.size else 1.0
            center_bonus = 1.0 - abs((xa + xb) / 2 - w / 2) / (w / 2)
            score = (a["len"] + b["len"]) * (0.5 + 0.5 * center_bonus) * (1.0 - min(density * 2, 0.7))
            cands.append({"x0": int(min(xa, xb)), "x1": int(max(xa, xb)),
                          "y0": int(top), "y1": int(bot), "inner_density": round(density, 3),
                          "score": round(float(score), 1)})
    cands.sort(key=lambda c: -c["score"])
    # 非极大抑制: 去掉与更高分候选重叠>50%的
    kept = []
    for c in cands:
        ok = True
        for k in kept:
            ix = max(0, min(c["x1"], k["x1"]) - max(c["x0"], k["x0"]))
            iy = max(0, min(c["y1"], k["y1"]) - max(c["y0"], k["y0"]))
            inter = ix * iy
            a1 = (c["x1"] - c["x0"]) * (c["y1"] - c["y0"])
            a2 = (k["x1"] - k["x0"]) * (k["y1"] - k["y0"])
            if inter > 0.5 * min(a1, a2):
                ok = False
                break
        if ok:
            kept.append(c)
    return kept


def wall_floor(hs, h):
    for s in hs:
        if max(s["y1"], s["y2"]) >= h * 0.55:
            return {"x1": s["x1"], "y1": s["y1"], "x2": s["x2"], "y2": s["y2"],
                    "len_ratio": round(s["len"] / WORK_W, 2)}
    return None


def traversability(edges, gray):
    h, w = gray.shape
    sub_e, sub_g = edges[h // 2:, :], gray[h // 2:, :]
    ch, cw = sub_e.shape[0] // GRID_ROWS, w // GRID_COLS
    grid = []
    for r in range(GRID_ROWS):
        row = []
        for c in range(GRID_COLS):
            ce = sub_e[r * ch:(r + 1) * ch, c * cw:(c + 1) * cw]
            cg = sub_g[r * ch:(r + 1) * ch, c * cw:(c + 1) * cw]
            density = float(ce.mean()) / 255.0
            var = float(cg.var())
            row.append({"density": round(density, 4), "var": round(var, 0),
                        "free": bool(density < 0.06 and var > 200)})
        grid.append(row)
    return grid


def annotate(img, res):
    vis = img.copy()
    for k, c in enumerate(res["doors"][:3]):
        col = (0, 255, 0) if k == 0 else (0, 200, 255)
        cv2.rectangle(vis, (c["x0"], c["y0"]), (c["x1"], c["y1"]), col, 2)
        cv2.putText(vis, "door%d s=%.0f d=%.2f" % (k, c["score"], c["inner_density"]),
                    (c["x0"], max(c["y0"] - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1)
    wf = res["wall_floor"]
    if wf:
        cv2.line(vis, (wf["x1"], wf["y1"]), (wf["x2"], wf["y2"]), (255, 120, 0), 3)
    h, w = vis.shape[:2]
    ch, cw = (h // 2) // GRID_ROWS, w // GRID_COLS
    for r in range(GRID_ROWS):
        for c in range(GRID_COLS):
            cell = res["grid"][r][c]
            col = (0, 180, 0) if cell["free"] else (0, 0, 180)
            cv2.rectangle(vis, (c * cw, h // 2 + r * ch),
                          ((c + 1) * cw, h // 2 + (r + 1) * ch), col,
                          1 if cell["free"] else 2)
    return vis


def process(path):
    t0 = time.time()
    img = load_image(path)
    t_load = (time.time() - t0) * 1000

    t0 = time.time()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges, segs = detect_segments(gray)
    merged = merge_collinear(segs)
    vs, hs = classify(merged, gray.shape[0])
    doors = pair_doors(vs, gray.shape[1], gray.shape[0], edges)
    wf = wall_floor(hs, gray.shape[0])
    grid = traversability(edges, gray)
    t_pipe = (time.time() - t0) * 1000

    res = {"doors": doors, "wall_floor": wf, "grid": grid,
           "n_raw": len(segs), "n_merged": len(merged), "n_vlines": len(vs),
           "timing_ms": {"load": round(t_load, 1), "pipeline": round(t_pipe, 1)}}
    base = os.path.splitext(os.path.basename(path))[0]
    out = os.path.join(os.path.dirname(os.path.abspath(path)), base + "_v2.jpg")
    cv2.imwrite(out, annotate(img, res))
    return out, res


def main():
    paths = sys.argv[1:]
    report = {}
    for p in paths:
        try:
            out, res = process(p)
            report[p] = res
            print("\n=== %s ===" % os.path.basename(p))
            print(json.dumps(res, ensure_ascii=False))
        except Exception as e:
            print("FAIL %s: %s" % (p, e))
    with open(os.path.join(os.path.dirname(os.path.abspath(paths[0])), "report_v2.json"), "w") as f:
        json.dump(report, f, ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
