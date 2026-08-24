#!/usr/bin/env python3
"""
fusion_test.py —— YOLO+OpenCV 融合第零步: 单帧对齐实验
只读消费现有服务, 不碰任何源文件:
  - http://127.0.0.1:8080/            MJPEG 抓单帧 (等于一个浏览器客户端)
  - http://127.0.0.1:8080/api/detections   YOLO bbox JSON
输出: fusion/grid_snapshot.json + fusion/fusion_annotated.jpg

设计原则(按 [世界] 线讨论定稿):
  - 网格格子标签是通用的: free/blocked/region:<name>/obj:<class>
  - YOLO 的任何类别都走同一个 obj: 槽位 (person/bottle/将来的冰箱...)
  - OpenCV 几何区域(门/墙地交界)走 region: 槽位, 门只是其中一个
  - 优先级: obj > region > blocked > free
"""
import cv2
import numpy as np
import json
import sys
import os
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import door_v3
import params as P

BASE = "http://127.0.0.1:8080"
OUT_DIR = os.path.dirname(os.path.abspath(__file__))


def grab_mjpeg_frame(timeout=8.0):
    """从 MJPEG 流抓一帧 (multipart/x-mixed-replace, boundary=frame)"""
    req = urllib.request.Request(BASE + "/", headers={"Range": "bytes=0-"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        buf = b""
        t0 = time.time()
        while time.time() - t0 < timeout:
            chunk = r.read(4096)
            if not chunk:
                break
            buf += chunk
            # JPEG 结束标记
            e = buf.find(b"\xff\xd9")
            if e != -1:
                s = buf.find(b"\xff\xd8")
                if s != -1 and s < e:
                    img = cv2.imdecode(np.frombuffer(buf[s:e + 2], np.uint8),
                                       cv2.IMREAD_COLOR)
                    if img is not None:
                        return img
                    buf = buf[e + 2:]
    raise IOError("no frame from mjpeg stream")


def fetch_detections(timeout=5.0):
    with urllib.request.urlopen(BASE + "/api/detections", timeout=timeout) as r:
        return json.loads(r.read().decode())


def cell_of(x, y, frame_w, frame_h):
    """像素 -> 底半区网格行列 (与 traversability 同一划分)"""
    col = int(x / frame_w * P.GRID_COLS)
    row = int((y - frame_h / 2) / (frame_h / 2) * P.GRID_ROWS)
    return (max(0, min(P.GRID_ROWS - 1, row)), max(0, min(P.GRID_COLS - 1, col)))


def main():
    t0 = time.time()
    frame = grab_mjpeg_frame()
    t_frame = (time.time() - t0) * 1000
    fh, fw = frame.shape[:2]

    det = fetch_detections()
    t_det = (time.time() - t0) * 1000

    # YOLO 推理帧与推流帧同源, 坐标按抓取帧尺寸缩放
    objs = []
    for d in det.get("detections", []):
        objs.append({"cls": d["c"], "conf": d["p"],
                     "bbox": [d["x1"], d["y1"], d["x2"], d["y2"]]})

    # OpenCV 几何 (v3 冻结管线)
    t1 = time.time()
    s = P.WORK_W / fw
    work = cv2.resize(frame, (P.WORK_W, int(fh * s)))
    gray = cv2.cvtColor(work, cv2.COLOR_BGR2GRAY)
    edges, segs = door_v3.detect_segments(gray)
    merged = door_v3.merge_collinear(segs)
    vs = door_v3.classify(merged, gray.shape[0])
    doors = door_v3.pair_doors(vs, gray.shape[1], gray.shape[0], edges)
    grid = door_v3.traversability(edges, gray)
    t_geo = (time.time() - t1) * 1000

    # --- 融合: 标签优先级 obj > region > blocked > free ---
    labels = [[{"label": "free" if grid[r][c]["free"] else "blocked",
                "sources": ["edge_density"]} for c in range(P.GRID_COLS)]
              for r in range(P.GRID_ROWS)]
    for d in doors:
        cx, cy = (d["x0"] + d["x1"]) / 2, (d["y0"] + d["y1"]) / 2
        r, c = cell_of(cx, cy, P.WORK_W, work.shape[0])
        labels[r][c] = {"label": "region:door", "sources": ["hough_geometry"],
                        "door_score": d["score"]}
    for o in objs:
        x1, y1, x2, y2 = [v * s for v in o["bbox"]]
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        r, c = cell_of(cx, cy, P.WORK_W, work.shape[0])
        labels[r][c] = {"label": "obj:%s" % o["cls"], "sources": ["yolo"],
                        "conf": round(o["conf"], 2)}

    snapshot = {
        "ts": time.time(),
        "frame_size": [fw, fh],
        "yolo": {"fps": det.get("fps"), "infer_ms": det.get("stats", {}).get("infer_ms"),
                 "objects": objs},
        "geometry": {"doors": doors, "pipeline_ms": round(t_geo, 1)},
        "grid": labels,
        "timing_ms": {"frame_grab": round(t_frame, 1), "det_fetch": round(t_det, 1),
                      "geometry": round(t_geo, 1), "total": round((time.time() - t0) * 1000, 1)},
    }

    # --- 标注图 ---
    vis = work.copy()
    gh, gw = vis.shape[:2]
    ch, cw = (gh // 2) // P.GRID_ROWS, gw // P.GRID_COLS
    colors = {"free": (0, 180, 0), "blocked": (0, 0, 180)}
    for r in range(P.GRID_ROWS):
        for c in range(P.GRID_COLS):
            lab = labels[r][c]["label"]
            if lab.startswith("obj:"):
                col, th = (0, 255, 255), 2
            elif lab.startswith("region:"):
                col, th = (255, 0, 255), 2
            else:
                col, th = colors.get(lab, (200, 200, 200)), 1
            cv2.rectangle(vis, (c * cw, gh // 2 + r * ch), ((c + 1) * cw, gh // 2 + (r + 1) * ch), col, th)
            if lab != "free":
                cv2.putText(vis, lab.split(":")[-1][:8], (c * cw + 4, gh // 2 + r * ch + 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, col, 1)
    for d in doors:
        cv2.rectangle(vis, (d["x0"], d["y0"]), (d["x1"], d["y1"]), (255, 0, 255), 2)
        cv2.putText(vis, "door %.0f" % d["score"], (d["x0"], max(d["y0"] - 5, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 0, 255), 1)
    for o in objs:
        x1, y1, x2, y2 = [int(v * s) for v in o["bbox"]]
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 255), 2)
        cv2.putText(vis, "%s %.2f" % (o["cls"], o["conf"]), (x1, max(y1 - 5, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)
    cv2.imwrite(os.path.join(OUT_DIR, "fusion_annotated.jpg"), vis)

    with open(os.path.join(OUT_DIR, "grid_snapshot.json"), "w") as f:
        json.dump(snapshot, f, ensure_ascii=False, indent=1)
    print(json.dumps(snapshot, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
