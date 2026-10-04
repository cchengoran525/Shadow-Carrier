#!/usr/bin/env python3
"""
geometry_daemon.py —— [世界] Phase 2: 极坐标世界快照
每周期: MJPEG 抓帧 → OpenCV 几何(edge密度/门) + /api/detections(YOLO) →
        经 polar_mapper(位姿来自 calib_pose.json) 产出 8扇区×3环 极坐标网格
输出: grid.json (冻结接口草案 v1) + topdown.png (俯视可视化)

用法:
  python3 geometry_daemon.py once        # 单帧快照
  python3 geometry_daemon.py loop 60     # 连续60秒 (~1Hz)
  python3 geometry_daemon.py loop        # 永续
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
import door_v3
import params as P
import geometry_map
import sector_score

BASE_CTRL = "http://127.0.0.1:80"
BASE_STREAM = "http://127.0.0.1:8080"
OUT = os.path.dirname(os.path.abspath(__file__))
WORK_W = 640
TILT_WORK = 112.0          # 工作俯仰指令(跟随时 [云台] 固定 112; 启动时归位)
PAN_WORK = 90.0
FREE_DENS_MAX = 0.06
PATCH = 20                 # 俯视格子采样半径(px)
YW_IN = "/dev/shm/yw_in.jpg"     # YOLO-World 慢环输入帧
YW_OUT = "/dev/shm/yw_out.json"  # YOLO-World 慢环输出
YW_MAX_AGE_S = 20.0              # 慢环结果最大可用年龄
YW_CONF_MIN = 0.30
MAP_PATH = "/dev/shm/world_map.json"   # 滚动地图持久化
SCAN_ABORT_FLAG = "/home/kickpi/hri_logs/scan_abort.flag"  # HRI 中断保险丝
CALIB_RANGE = (0.0, 0.0)               # 本次标定扫掠覆盖的 tilt 区间(有效性判定)
POSE_SRC = "unknown"                   # 位姿来源: empirical/pnp/fallback


def load_pose_calib():
    """优先级: 生产经验标定(calib_prod.json, 已知距离解算) > PnP 扫掠拟合
    返回 (h_func, tilt_func, date, tmin, tmax, src)"""
    prod = os.path.join(OUT, "calib_prod.json")
    if os.path.exists(prod):
        with open(prod) as f:
            c = json.load(f)
        tc, hm, td = c["tilt_cmd"], c["h_m"], c["tilt_deg"]
        return (lambda cmd: hm if abs(cmd - tc) <= 2.0 else 0.11), \
               (lambda cmd: td if abs(cmd - tc) <= 2.0 else 0.0), \
               c.get("method", "empirical"), tc - 1.0, tc + 1.0, "empirical"
    path = os.path.join(OUT, "calib_pose.json")
    if os.path.exists(path):
        with open(path) as f:
            c = json.load(f)
        fk, fb = c["fit_h"]["k"], c["fit_h"]["b"]
        tk, tb = c["fit_tilt"]["k"], c["fit_tilt"]["b"]
        samples = [s["tilt_cmd"] for s in c.get("samples", [])]
        tmin = min(samples) if samples else 0
        tmax = max(samples) if samples else 0
        return (lambda cmd: (fk * cmd + fb) / 1000.0), \
               (lambda cmd: tk * cmd + tb), c.get("date"), tmin, tmax, "pnp"
    return (lambda cmd: 0.100), (lambda cmd: 1.2921 * cmd - 145.2), "fallback", 0, 0, "fallback"


def gimbal(pan=None, tilt=None):
    body = {}
    if pan is not None:
        body["pan"] = pan
    if tilt is not None:
        body["tilt"] = tilt
    req = urllib.request.Request(BASE_CTRL + "/gimbal",
                                 data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status == 200
    except Exception:
        return False


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


def fetch_detections(timeout=5.0):
    with urllib.request.urlopen(BASE_STREAM + "/api/detections", timeout=timeout) as r:
        return json.loads(r.read().decode())


def ground_to_pixel(x, y, h, tilt_down_deg):
    """地面点(车体系x前y右,相对相机) -> 像素; 不在画内返回 None"""
    t = math.radians(tilt_down_deg)
    xc = x * math.cos(t) + h * math.sin(t)
    if xc <= 1e-6:
        return None
    zc = -x * math.sin(t) + h * math.cos(t)
    u = pm.FX * y / xc + pm.CX
    v = pm.FY * zc / xc + pm.CY
    return u, v


def cell_center_polar(sector, ring):
    b = sector * 45.0
    lo, hi = pm.ring_range(ring)
    d = (lo + hi) / 2 if math.isfinite(hi) else lo + 1.0
    return b, d


class WorldSnapshot:
    def __init__(self, h_func, tilt_func):
        self.h_func, self.tilt_func = h_func, tilt_func

    def capture(self, pan_cmd=PAN_WORK, do_doors=True):
        t0 = time.time()
        frame = grab_mjpeg_frame()
        fh, fw = frame.shape[:2]
        s = WORK_W / fw
        work = cv2.resize(frame, (WORK_W, int(fh * s)))
        # 喂给 YOLO-World 慢环(异步: 写入后本轮先用上一轮结果)
        try:
            cv2.imwrite(YW_IN, frame)
        except Exception:
            pass
        gray = cv2.cvtColor(work, cv2.COLOR_BGR2GRAY)
        edges, segs = door_v3.detect_segments(gray)
        doors = []
        if do_doors:
            merged = door_v3.merge_collinear(segs)
            vsegs = door_v3.classify(merged, gray.shape[0])
            doors = door_v3.pair_doors(vsegs, gray.shape[1], gray.shape[0], edges)
        det = fetch_detections()
        t_geo = (time.time() - t0) * 1000

        h = self.h_func(TILT_WORK)
        tilt_dn = self.tilt_func(TILT_WORK)

        # --- YOLO 物体: bbox 底边中点 = 落地点 ---
        objs = []
        for d_ in det.get("detections", []):
            u = (d_["x1"] + d_["x2"]) / 2 * s
            v = d_["y2"] * s
            b, dist = pm.pixel_to_polar(u, v, TILT_WORK, pan_cmd, h_cam=h, tilt_deg=tilt_dn)
            if b is None:
                continue
            sector, ring = pm.polar_to_sector(b, dist)
            objs.append({"cls": d_["c"], "conf": round(d_["p"], 2),
                         "bearing_deg": round(b, 1), "dist_m": round(dist, 2),
                         "sector": sector, "ring": ring, "source": "yolov8n"})

        # --- 门(几何) ---
        door_objs = []
        for d_ in doors:
            u = (d_["x0"] + d_["x1"]) / 2
            v = d_["y1"]
            b, dist = pm.pixel_to_polar(u, v, TILT_WORK, pan_cmd, h_cam=h, tilt_deg=tilt_dn)
            if b is None:
                continue
            sector, ring = pm.polar_to_sector(b, dist)
            door_objs.append({"cls": "door", "conf": round(d_["score"] / 1000, 2),
                              "bearing_deg": round(b, 1), "dist_m": round(dist, 2),
                              "sector": sector, "ring": ring, "source": "hough"})

        # --- YOLO-World 慢语义环 (异步结果, 容忍一轮延迟) ---
        yw_objs = []
        yw_meta = None
        try:
            if os.path.exists(YW_OUT):
                age = time.time() - os.path.getmtime(YW_OUT)
                if age <= YW_MAX_AGE_S:
                    with open(YW_OUT) as f:
                        ywd = json.load(f)
                    yw_meta = {"infer_ms": ywd.get("infer_ms"), "age_s": round(age, 1)}
                    for d_ in ywd.get("objects", []):
                        if d_["conf"] < YW_CONF_MIN:
                            continue
                        u = (d_["bbox"][0] + d_["bbox"][2]) / 2 * s
                        v = d_["bbox"][3] * s
                        b, dist = pm.pixel_to_polar(u, v, TILT_WORK, pan_cmd, h_cam=h, tilt_deg=tilt_dn)
                        if b is None:
                            continue
                        sector, ring = pm.polar_to_sector(b, dist)
                        yw_objs.append({"cls": d_["cls"], "conf": round(d_["conf"], 2),
                                        "bearing_deg": round(b, 1), "dist_m": round(dist, 2),
                                        "sector": sector, "ring": ring, "source": "yolo_world"})
        except Exception:
            pass

        # 多源去重: 同 (cls,扇区,环) 保留高置信
        dedup = {}
        for o in objs + yw_objs:
            k = (o["cls"], o["sector"], o["ring"])
            if k not in dedup or o["conf"] > dedup[k]["conf"]:
                dedup[k] = o
        objs = list(dedup.values())

        # --- 极坐标网格: 每格中心反投像素, 采边缘密度 ---
        grid = []
        for ring in range(3):
            row = []
            for sector in range(8):
                b, d = cell_center_polar(sector, ring)
                x = d * math.cos(math.radians(b))
                y = d * math.sin(math.radians(b))
                uv = ground_to_pixel(x, y, h, tilt_dn)
                if uv is None or not (0 <= uv[0] < WORK_W and 0 <= uv[1] < gray.shape[0]):
                    row.append({"label": "unknown", "sources": []})
                    continue
                u, v = int(uv[0]), int(uv[1])
                patch = edges[max(0, v - PATCH):v + PATCH, max(0, u - PATCH):u + PATCH]
                if patch.size == 0:
                    row.append({"label": "unknown", "sources": []})
                    continue
                dens = float(patch.mean()) / 255.0
                row.append({"label": "free" if dens < FREE_DENS_MAX else "blocked",
                            "sources": ["edge_density"],
                            "density": round(dens, 4)})
            grid.append(row)

        # 格子标签被物体覆盖(物体比自由空间可信)
        for o in objs + door_objs:
            grid[o["ring"]][o["sector"]] = {
                "label": "obj:%s" % o["cls"], "sources": ["yolo" if o["cls"] != "door" else "hough"],
                "bearing": o["bearing_deg"], "dist_m": o["dist_m"]}

        free_dirs = [s2 * 45 for s2 in range(8)
                     if grid[0][s2]["label"] == "free"]

        # 有效性: 工作角在扫掠覆盖内 -> 直接可信; 否则允许有限外推并显式标记
        in_range = CALIB_RANGE[1] and (CALIB_RANGE[0] - 2.0) <= TILT_WORK <= (CALIB_RANGE[1] + 2.0)
        extrap = bool(CALIB_RANGE[1]) and not in_range and \
            (CALIB_RANGE[0] - 25.0) <= TILT_WORK <= (CALIB_RANGE[1] + 25.0)
        dist_ok = bool(in_range or extrap)
        return {
            "ts": time.time(),
            "pose": {"tilt_cmd": TILT_WORK, "pan_cmd": PAN_WORK,
                     "h_m": round(h, 4), "tilt_deg": round(tilt_dn, 2),
                     "pose_source": POSE_SRC,
                     "dist_calibrated": dist_ok, "dist_extrapolated": bool(extrap)},
            "objects": objs + door_objs,
            "grid_polar": grid,
            "free_directions_deg": free_dirs,
            "yolo_world": yw_meta,
            "timing_ms": {"capture_geo": round(t_geo, 1),
                          "total": round((time.time() - t0) * 1000, 1)},
        }, work


def draw_topdown(snap):
    top = np.full((640, 640, 3), 255, np.uint8)
    cx = cy = 320

    def P(bear, dist):
        r = int(min(dist / 3.2, 1.0) * 280)
        a = math.radians(bear - 90)   # 0°朝上
        return (int(cx + r * math.cos(a + math.pi / 2) * 0 + r * math.sin(a + math.pi / 2)),
                int(cy - r * math.cos(a)))

    # 距离环
    for i, rmax in enumerate(pm.RINGS_M):
        r = int(rmax / 3.2 * 280)
        cv2.circle(top, (cx, cy), r, (210, 210, 210), 1)
        cv2.putText(top, "%.1fm" % rmax, (cx + 3, cy - r + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1)
    for ring in range(3):
        for sector in range(8):
            cell = snap["grid_polar"][ring][sector]
            lab = cell["label"]
            if lab == "free":
                col, thick = (0, 200, 0), -1
            elif lab == "blocked":
                col, thick = (0, 0, 200), -1
            elif lab == "unknown":
                col, thick = (230, 230, 230), -1
            elif lab.startswith("obj:"):
                col, thick = (0, 220, 220), -1
            else:
                col, thick = (255, 0, 255), -1
            a0 = math.radians(sector * 45 - 22.5 - 90)
            a1 = math.radians(sector * 45 + 22.5 - 90)
            r0 = 0 if ring == 0 else int(pm.RINGS_M[ring - 1] / 3.2 * 280)
            r1 = int(pm.RINGS_M[ring] / 3.2 * 280)
            pts = [P(0, 0)] if ring == 0 else None
            p1 = (int(cx + r1 * math.cos(a0)), int(cy + r1 * math.sin(a0)))
            p2 = (int(cx + r1 * math.cos(a1)), int(cy + r1 * math.sin(a1)))
            if ring == 0:
                cv2.fillPoly(top, [np.array([[cx, cy], p1, p2])], col)
            else:
                p3 = (int(cx + r0 * math.cos(a1)), int(cy + r0 * math.sin(a1)))
                p4 = (int(cx + r0 * math.cos(a0)), int(cy + r0 * math.sin(a0)))
                cv2.fillPoly(top, [np.array([p1, p2, p3, p4])], col)
    for o in snap["objects"]:
        r = int(min(o["dist_m"] / 3.2, 1.0) * 280)
        a = math.radians(o["bearing_deg"] - 90)
        px = int(cx + r * math.sin(a + math.pi / 2))
        py = int(cy - r * math.cos(a))
        cv2.circle(top, (px, py), 4, (0, 0, 0), 2)
        cv2.putText(top, o["cls"][:8], (px + 5, py), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 1)
    cv2.rectangle(top, (cx - 6, cy - 6), (cx + 6, cy + 6), (0, 0, 0), -1)
    return top


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "once"
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else None
    h_func, tilt_func, calib_date, tmin, tmax, calib_src = load_pose_calib()
    global CALIB_RANGE, POSE_SRC
    CALIB_RANGE = (tmin, tmax)
    POSE_SRC = calib_src
    print("pose calib: %s [%s] | h(%.0f)=%.1fmm tilt(%.0f)=%.2f° | sweep范围[%.0f,%.0f]"
          % (calib_date, calib_src, TILT_WORK, h_func(TILT_WORK) * 1000, TILT_WORK,
             tilt_func(TILT_WORK), tmin, tmax))
    snap_engine = WorldSnapshot(h_func, tilt_func)

    if mode == "once":
        gimbal(pan=PAN_WORK, tilt=TILT_WORK)
        time.sleep(1.0)
        snap, work = snap_engine.capture()
        m = geometry_map.RollingMap.load(MAP_PATH).update(snap)
        m.save(MAP_PATH)
        snap["map"] = m.summary()
        snap["sector_scores"] = sector_score.score_map(snap["map"])
        with open(os.path.join(OUT, "grid.json"), "w") as f:
            json.dump(snap, f, ensure_ascii=False, indent=1)
        cv2.imwrite(os.path.join(OUT, "topdown.png"), draw_topdown(snap))
        print(json.dumps(snap, ensure_ascii=False, indent=1))
    elif mode == "scan":
        """静止时刻专用: pan 扫掠三帧拼接. FOLLOW 模式禁止调用!
        安全: 启动前/每帧间检查 HRI 中断保险丝; 结束归位到 [云台] 协议值 (PAN 90.58/TLT 112)"""
        if os.path.exists(SCAN_ABORT_FLAG):
            print("scan aborted: abort flag present -> %s" % SCAN_ABORT_FLAG)
            return
        pans = [45.0, 90.0, 135.0]
        snaps = []
        for p in pans:
            if os.path.exists(SCAN_ABORT_FLAG):
                print("scan aborted mid-way at pan=%.0f (flag)" % p)
                gimbal(pan=90.58, tilt=112)      # [云台] 协议归位
                return
            gimbal(pan=p, tilt=TILT_WORK)
            time.sleep(1.2)
            snap, work = snap_engine.capture(pan_cmd=p)
            snaps.append(snap)
            print("pan=%.0f: %d objs" % (p, len(snap["objects"])), flush=True)
        gimbal(pan=90.58, tilt=112)   # 归位: [云台] 协议值

        # 合并: 逐格取信息量最大者; free与blocked并存取blocked(保守)
        RANK = {"unknown": 0, "free": 1, "blocked": 2}
        grid = []
        for ring in range(3):
            row = []
            for sector in range(8):
                cands = [s2["grid_polar"][ring][sector] for s2 in snaps]
                best = max(cands, key=lambda c: RANK.get(c["label"].split(":")[0]
                                                         if c["label"].startswith(("obj:", "region:"))
                                                         else c["label"], 3))
                row.append(best)
            grid.append(row)
        objects = []
        seen = set()
        for s2 in snaps:
            for o in s2["objects"]:
                key = (o["cls"], o["sector"], o["ring"])
                if key not in seen:
                    seen.add(key)
                    objects.append(o)
        free_dirs = sorted({s2 * 45 for s2 in range(8) if grid[0][s2]["label"] == "free"})
        snap = {"ts": time.time(), "scan": True, "scan_pans": pans,
                "pose": snaps[1]["pose"],
                "objects": objects, "grid_polar": grid,
                "free_directions_deg": free_dirs,
                "timing_ms": {"n_frames": len(snaps)}}
        m = geometry_map.RollingMap.load(MAP_PATH).update(snap)
        m.save(MAP_PATH)
        snap["map"] = m.summary()
        snap["sector_scores"] = sector_score.score_map(snap["map"])
        with open(os.path.join(OUT, "grid.json"), "w") as f:
            json.dump(snap, f, ensure_ascii=False, indent=1)
        cv2.imwrite(os.path.join(OUT, "topdown.png"), draw_topdown(snap))
        print("scan merged: %d objs, free=%s" % (len(objects), free_dirs))
    elif mode == "loop":
        gimbal(pan=PAN_WORK, tilt=TILT_WORK)
        time.sleep(1.0)
        t_end = time.time() + dur if dur else None
        n = 0
        m = geometry_map.RollingMap.load(MAP_PATH)
        # P1-④: 门检测(贵)与网格(便宜)分离节拍; 慢时自动降门检测频率, 滚动地图兜时效
        door_interval = 1.0
        last_doors = 0.0
        while t_end is None or time.time() < t_end:
            try:
                now = time.time()
                do_doors = (now - last_doors) >= door_interval
                t_c0 = time.time()
                snap, work = snap_engine.capture(do_doors=do_doors)
                if do_doors:
                    last_doors = now
                dt = time.time() - t_c0
                # 自适应: 单轮 >1.5s 则门检测按 2×dt 跳帧; <0.8s 回到 1s
                if dt > 1.5:
                    door_interval = max(3.0, dt * 2.0)
                elif dt < 0.8:
                    door_interval = 1.0
                m.update(snap)
                m.save(MAP_PATH)
                snap["map"] = m.summary()
                snap["sector_scores"] = sector_score.score_map(snap["map"])
                snap["timing_ms"]["door_interval_s"] = door_interval
                with open(os.path.join(OUT, "grid.json.tmp"), "w") as f:
                    json.dump(snap, f, ensure_ascii=False, indent=1)
                os.replace(os.path.join(OUT, "grid.json.tmp"), os.path.join(OUT, "grid.json"))
                cv2.imwrite(os.path.join(OUT, "topdown.png"), draw_topdown(snap))
                n += 1
                objs = ", ".join("%s@%.1f°/%.2fm" % (o["cls"], o["bearing_deg"], o["dist_m"])
                                 for o in snap["objects"]) or "none"
                print("[%d] %s | %.2fs | doors=%s | objs: %s"
                      % (n, time.strftime("%H:%M:%S"), dt,
                         "on" if do_doors else "skip", objs), flush=True)
            except Exception as e:
                print("cycle error: %s" % e, flush=True)
            time.sleep(1.0)
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
