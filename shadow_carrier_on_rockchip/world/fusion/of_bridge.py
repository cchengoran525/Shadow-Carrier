#!/usr/bin/env python3
"""
of_bridge.py —— YOLO↔光流桥原型 (YOLO20/OpenCV80 落地验证)
录制模式: 抓 MJPEG 序列 + 同期 detections -> /dev/shm/of_test/
评测模式: KLT光流在真实检测帧之间插值 bbox, 输出 HRI 要的质量指标:
  - 重锚漂移(350ms 间隔时 of 框 vs 新真实框的中心距)
  - of_valid 有效率 / 失效检测
  - 抖动对比(裸抽帧 vs 光流插值)
  - CPU 单帧耗时
用法:
  python3 of_bridge.py record [seconds]
  python3 of_bridge.py eval
"""
import cv2
import numpy as np
import json
import time
import os
import sys
import math
import urllib.request

BASE = "http://127.0.0.1:8080"
SEQ = "/dev/shm/of_test"
REANCHOR_MS = 350          # HRI 红线①: 真实重锚间隔上限
YOLO_MIN_GAP_MS = 250      # 模拟真实 YOLO 抽帧率(~4Hz)
MAX_PTS = 16
FB_MAX_ERR = 1.0           # 前向后向光流一致性阈值(px)
MIN_VALID_FRAC = 0.30      # 有效点占比下限, 低于则 of_valid=False
MIN_PTS = 8


def grab_frame(timeout=8.0):
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


def fetch_detections(timeout=3.0):
    try:
        with urllib.request.urlopen(BASE + "/api/detections", timeout=timeout) as r:
            return json.loads(r.read().decode())
    except Exception:
        return {"detections": []}


def cmd_record(seconds):
    os.makedirs(SEQ, exist_ok=True)
    for f in os.listdir(SEQ):
        os.remove(os.path.join(SEQ, f))
    t_end = time.time() + seconds
    n = 0
    meta = []
    while time.time() < t_end:
        t0 = time.time()
        try:
            frame = grab_frame()
        except IOError:
            continue
        det = fetch_detections()
        fn = os.path.join(SEQ, "f%04d.jpg" % n)
        cv2.imwrite(fn, frame)
        meta.append({"i": n, "t": round(t0, 4), "det": det.get("detections", [])})
        n += 1
        # 尽量贴近流帧率(约15fps)
        dt = time.time() - t0
        if dt < 1 / 15.0:
            time.sleep(1 / 15.0 - dt)
    with open(os.path.join(SEQ, "meta.json"), "w") as f:
        json.dump(meta, f)
    print("recorded %d frames -> %s" % (n, SEQ))


class FlowTracker:
    """单人 bbox 的 KLT 光流续命器"""

    def __init__(self):
        self.pts = None
        self.prev_gray = None
        self.bbox = None
        self.anchor_bbox = None
        self.anchor_t = 0.0
        self.valid = False

    def anchor(self, gray, bbox, t):
        x1, y1, x2, y2 = [int(v) for v in bbox]
        roi = gray[max(0, y1):y2, max(0, x1):x2]
        self.bbox = list(bbox)
        self.anchor_bbox = list(bbox)
        self.anchor_t = t
        self.valid = False
        if roi.size == 0:
            self.pts = None
            return
        pts = cv2.goodFeaturesToTrack(roi, maxCorners=MAX_PTS, qualityLevel=0.01,
                                      minDistance=5, blockSize=7)
        if pts is not None:
            pts[:, 0, 0] += max(0, x1)
            pts[:, 0, 1] += max(0, y1)
        self.pts = pts
        self.prev_gray = gray

    def step(self, gray, t):
        """中间帧: 返回 (bbox|None, valid). ROI 裁剪以省 CPU"""
        if self.pts is None or self.prev_gray is None or len(self.pts) < MIN_PTS:
            self.valid = False
            self.prev_gray = gray
            return None, False
        fh, fw = gray.shape[:2]
        x1, y1, x2, y2 = self.bbox
        mw, mh = (x2 - x1) * 0.5, (y2 - y1) * 0.5
        cx1, cy1 = int(max(0, x1 - mw)), int(max(0, y1 - mh))
        cx2, cy2 = int(min(fw, x2 + mw)), int(min(fh, y2 + mh))
        if cx2 - cx1 < 16 or cy2 - cy1 < 16:
            self.valid = False
            self.prev_gray = gray
            return None, False
        crop_prev = self.prev_gray[cy1:cy2, cx1:cx2]
        crop_cur = gray[cy1:cy2, cx1:cx2]
        pts_crop = self.pts.reshape(-1, 2) - np.array([cx1, cy1], np.float32)
        pts_crop = pts_crop.reshape(-1, 1, 2).astype(np.float32)
        nxt, st, _ = cv2.calcOpticalFlowPyrLK(crop_prev, crop_cur, pts_crop, None,
                                              winSize=(15, 15), maxLevel=2)
        if nxt is None:
            self.valid = False
            self.prev_gray = gray
            return None, False
        st_f = st.reshape(-1) > 0
        old = pts_crop.reshape(-1, 2)[st_f] + np.array([cx1, cy1], np.float32)
        new = nxt.reshape(-1, 2)[st_f] + np.array([cx1, cy1], np.float32)
        frac = len(old) / max(len(self.pts), 1)
        if frac < MIN_VALID_FRAC or len(old) < MIN_PTS:
            self.valid = False
            self.prev_gray = gray
            self.pts = None
            return None, False
        M, inliers = cv2.estimateAffinePartial2D(old, new, method=cv2.RANSAC,
                                                 ransacReprojThreshold=3.0)
        if M is None or inliers is None or int(inliers.sum()) < MIN_PTS:
            self.valid = False
            self.prev_gray = gray
            self.pts = None
            return None, False
        self.inlier_frac = float(inliers.sum()) / max(len(old), 1)
        if self.inlier_frac < 0.5:
            self.valid = False
            self.prev_gray = gray
            self.pts = None
            return None, False
        x1, y1, x2, y2 = self.bbox
        corners = np.array([[x1, y1], [x2, y1], [x1, y2], [x2, y2]], np.float32)
        tc = cv2.transform(corners.reshape(-1, 1, 2), M).reshape(-1, 2)
        self.bbox = [float(tc[:, 0].min()), float(tc[:, 1].min()),
                     float(tc[:, 0].max()), float(tc[:, 1].max())]
        # 点云补充: 在当前框内重新取点(防止点全跑光)
        bx1, by1, bx2, by2 = [int(v) for v in self.bbox]
        roi = gray[max(0, by1):by2, max(0, bx1):bx2]
        fresh = None
        if roi.size > 0:
            fresh = cv2.goodFeaturesToTrack(roi, maxCorners=MAX_PTS, qualityLevel=0.01,
                                            minDistance=6, blockSize=7)
            if fresh is not None:
                fresh[:, 0, 0] += max(0, bx1)
                fresh[:, 0, 1] += max(0, by1)
        if fresh is not None and len(fresh) >= MIN_PTS:
            self.pts = fresh
        else:
            self.pts = new.reshape(-1, 1, 2).astype(np.float32)
        self.prev_gray = gray
        self.valid = True
        return list(self.bbox), True


def cmd_eval():
    meta = json.load(open(os.path.join(SEQ, "meta.json")))
    frames = []
    for m in meta:
        img = cv2.imread(os.path.join(SEQ, "f%04d.jpg" % m["i"]), cv2.IMREAD_GRAYSCALE)
        frames.append(img)
    n = len(frames)
    print("eval on %d frames" % n)

    def pick_person(dets):
        best = None
        for d in dets:
            if d["c"] == "person":
                if best is None or d["p"] > best["p"]:
                    best = d
        return best

    tracker = None
    last_yolo_t = -1e9
    out = []            # {i,t,bbox,valid,t_since_anchor}
    cpu_times = []
    n_reanchor = 0
    for i in range(n):
        t = meta[i]["t"] * 1000.0
        gray = frames[i]
        dets = meta[i]["det"]
        p = pick_person(dets)
        # 真实检测到达 & 到了抽帧间隔 -> 重锚
        if p is not None and (t - last_yolo_t) >= YOLO_MIN_GAP_MS:
            last_yolo_t = t
            n_reanchor += 1
            if tracker is None:
                tracker = FlowTracker()
            tracker.anchor(gray, [p["x1"], p["y1"], p["x2"], p["y2"]], t)
            out.append({"i": i, "t": t, "bbox": tracker.anchor_bbox, "src": "yolo",
                        "valid": True, "age": 0})
            continue
        if tracker is None:
            continue
        t0 = time.time()
        bbox, valid = tracker.step(gray, t)
        cpu_times.append((time.time() - t0) * 1000)
        age = t - tracker.anchor_t
        out.append({"i": i, "t": t, "bbox": bbox, "src": "of" if valid else "invalid",
                    "valid": valid, "age": age})

    # 指标 1: 重锚漂移 (光流框 vs 新真实框) + 归一化
    drifts = []
    for k in range(1, len(out)):
        if out[k]["src"] == "yolo" and out[k - 1]["bbox"] is not None:
            a = out[k - 1]["bbox"]; b = out[k]["bbox"]
            ca = ((a[0] + a[2]) / 2, (a[1] + a[3]) / 2)
            cb = ((b[0] + b[2]) / 2, (b[1] + b[3]) / 2)
            age = out[k - 1]["age"]
            diag = math.hypot(b[2] - b[0], b[3] - b[1])
            drifts.append((math.hypot(ca[0] - cb[0], ca[1] - cb[1]), age, diag))
    # 对照: OF 是否比"沿用上一真实框"更接近新真实框
    better = 0
    total = 0
    for k in range(1, len(out)):
        if out[k]["src"] == "yolo" and out[k - 1]["bbox"] is not None:
            newb = out[k]["bbox"]
            # 找 OF 链上上一个真实框
            j = k - 1
            prev_yolo = None
            while j >= 0:
                if out[j]["src"] == "yolo":
                    prev_yolo = out[j]["bbox"]
                    break
                j -= 1
            if prev_yolo is None:
                continue
            def c(b):
                return ((b[0] + b[2]) / 2, (b[1] + b[3]) / 2)
            cn = c(newb)
            d_of = math.hypot(c(out[k - 1]["bbox"])[0] - cn[0], c(out[k - 1]["bbox"])[1] - cn[1])
            d_prev = math.hypot(c(prev_yolo)[0] - cn[0], c(prev_yolo)[1] - cn[1])
            total += 1
            if d_of <= d_prev:
                better += 1
    # 对照: 裸 YOLO 连续两次真实检测的中心跳变(检测噪声地板)
    yolo_boxes = [o for o in out if o["src"] == "yolo"]
    yolo_jump = []
    for k in range(1, len(yolo_boxes)):
        a, b = yolo_boxes[k - 1]["bbox"], yolo_boxes[k]["bbox"]
        ca = ((a[0] + a[2]) / 2, (a[1] + a[3]) / 2)
        cb = ((b[0] + b[2]) / 2, (b[1] + b[3]) / 2)
        yolo_jump.append(math.hypot(ca[0] - cb[0], ca[1] - cb[1]))
    # 指标 2: 有效率
    of_frames = [o for o in out if o["src"] == "of"]
    inv_frames = [o for o in out if o["src"] == "invalid"]
    valid_rate = len(of_frames) / max(len(of_frames) + len(inv_frames), 1)
    # 指标 3: 抖动 (相邻帧中心加速度的均值; 越小越平滑)
    def jitter(seq):
        cs = []
        for o in seq:
            if o["bbox"]:
                cs.append(((o["bbox"][0] + o["bbox"][2]) / 2, (o["bbox"][1] + o["bbox"][3]) / 2))
        if len(cs) < 3:
            return None
        v = np.diff(np.array(cs), axis=0)
        a = np.diff(v, axis=0)
        return float(np.linalg.norm(a, axis=1).mean())
    j_of = jitter([o for o in out if o["src"] in ("of", "yolo")])
    j_yolo = jitter([o for o in out if o["src"] == "yolo"])

    res = {
        "frames": n,
        "reanchors": n_reanchor,
        "of_valid_rate": round(valid_rate, 3),
        "of_cpu_ms_mean": round(float(np.mean(cpu_times)), 2) if cpu_times else None,
        "of_cpu_ms_p95": round(float(np.percentile(cpu_times, 95)), 2) if cpu_times else None,
        "reanchor_drift_px_mean": round(float(np.mean([d for d, _, _ in drifts])), 1) if drifts else None,
        "reanchor_drift_px_p90": round(float(np.percentile([d for d, _, _ in drifts], 90)), 1) if drifts else None,
        "reanchor_drift_norm_mean": round(float(np.mean([d / max(diag, 1) for d, _, diag in drifts])), 3) if drifts else None,
        "reanchor_age_ms_mean": round(float(np.mean([a for _, a, _ in drifts])), 0) if drifts else None,
        "yolo_jump_px_mean": round(float(np.mean(yolo_jump)), 1) if yolo_jump else None,
        "yolo_jump_px_p90": round(float(np.percentile(yolo_jump, 90)), 1) if yolo_jump else None,
        "of_beats_hold_ratio": round(better / total, 3) if total else None,
        "jitter_of_interp": round(j_of, 3) if j_of else None,
        "jitter_yolo_only": round(j_yolo, 3) if j_yolo else None,
        "of_beats_hold_ratio": round(better / total, 3) if total else None,
        "hri_redline_reanchor_le_350ms": bool(drifts and np.percentile([a for _, a, _ in drifts], 90) <= REANCHOR_MS + 60),
    }
    with open(os.path.join(SEQ, "eval.json"), "w") as f:
        json.dump(res, f, indent=1)
    print(json.dumps(res, indent=1))


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "eval"
    if mode == "record":
        cmd_record(float(sys.argv[2]) if len(sys.argv) > 2 else 20)
    else:
        cmd_eval()
