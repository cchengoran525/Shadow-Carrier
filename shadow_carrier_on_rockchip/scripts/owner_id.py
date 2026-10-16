#!/usr/bin/env python3
"""owner_id.py v0 - 纯视觉快速认主: 进入跟随模式时抓取 HSV 颜色 + 体态比模板"""
import time, json, urllib.request, threading
import cv2
import numpy as np

DET_API = "http://127.0.0.1:8080/api/detections"
FRAME_A = "/dev/shm/yolo_frame_a.jpg"
FRAME_B = "/dev/shm/yolo_frame_b.jpg"

ENROLL_DURATION_S = 3.0   # 认主采样时长
ENROLL_INTERVAL_S = 0.25  # 采样间隔 (~12个样本)

# HSV 直方图参数
H_BINS, S_BINS = 30, 32

# 打分权重与阈值
W_COLOR = 0.7
W_ASPECT = 0.3
SCORE_MIN = 0.40         # 低于此分视为不是主人
SAT_LOW = 40.0           # 模板平均饱和度低于此值 → 颜色权重自动降档
# 注: 不设饱和度/宽高比采样质量门。
# - "跟错桌子"根因是云台坏机位的半身采样, 归[云台]线治理;
# - 体态比(aspect)的解释权归[HRI](姿态判断), 认主端只读原始值, 不定义语义;
# - 坏模板的后果已被L2兜住(失配→wait_owner/lost_stop), 无需前置拒绝。
CONF_MIN = 0.50          # person置信度地板(YOLO对家具的低置信度误报在此被拦)
MIN_BOX_H = 70           # 最小框高px(过滤远处小误检)

# ===== 底盘转向"弱线索" (2026-10-16) =====
# 场景: 主人近距擦身时画面里只剩腿(无头无躯干), YOLO 出不了 person → 若"没有人框就停",
# 车会在demo里跟丢。这条通道只回答"往哪边转", 不回答"谁是主人"。
# ⚠️ 机主原则第0层(见 _fetch_detections): 候选池入口不放宽; 本通道独立于候选池,
#    不进 select_target / 不写 owner.json / 不进 KF / 不放行前进(只授权原地转)。
LOWCONF_MIN_P = 0.30     # 低置信框下限: 低于此连弱线索都不算(幻影兜底)
LEG_BAND_TOP_FRAC = 0.45 # "下半带"起始行比例(地平线以下; 腿/下装在这里)
LEG_BP_THRESH = 60       # 颜色反投影二值化阈值
LEG_MIN_AREA = 300       # 连通域最小面积(px²)
LEG_MIN_ASPECT = 0.5     # 高/宽 下限: 腿是竖条, 横比竖宽的块多半是地面/墙裙

# ===== BLE 在场心跳 (慢速兜底, 非实时锚定) =====
BAND_MAC = "04:34:C3:15:AE:0E"   # Xiaomi Smart Band 9, 未连接手机时公开广播
BAND_FRESH_S = 60                # 多久内见过手环算"主人还在附近"

_band_last_seen = 0.0
_heartbeat_started = False

# ===== WiFi 热点在场信号 (方案A): 手机连着车热点 = 主人在场 =====
# Android/iOS 的WiFi随机MAC按SSID固定 → 同一手机每次连同一热点的MAC一致
# 注意: AP6255 AP模式下 iw station dump 无输出(驱动限制), 改用 邻居表+ping 判活
WIFI_FRESH_S = 60
PHONE_MAC = "b6:ac:ee:7e:b2:fd"   # iPhone8 在 ShadowCarrier-RK 热点下的固定MAC
_wifi_last_seen = 0.0


def _wifi_station_check(log=print):
    """按MAC在邻居表找手机的IP, ping通即在场"""
    global _wifi_last_seen
    import subprocess
    try:
        out = subprocess.run(["ip", "neigh", "show"],
                             capture_output=True, text=True, timeout=5).stdout
        for line in out.splitlines():
            # 格式: 192.168.4.4 dev wlan0 lladdr b6:ac:..:fd REACHABLE
            parts = line.split()
            if len(parts) >= 6 and parts[1] == "dev" and \
               parts[3] == "lladdr" and parts[4].lower() == PHONE_MAC.lower():
                if parts[5].upper() in ("FAILED", "INCOMPLETE"):
                    continue
                r = subprocess.run(["ping", "-c1", "-W1", parts[0]],
                                   capture_output=True, timeout=3)
                if r.returncode == 0:
                    _wifi_last_seen = time.time()
                    return True
    except Exception:
        pass
    return False


def start_ble_heartbeat(log=print):
    """后台线程: 每~40s扫一轮bluetoothctl, 记录手环最后出现时间"""
    global _heartbeat_started
    if _heartbeat_started:
        return
    _heartbeat_started = True

    def _loop():
        import re, subprocess
        pat = re.compile(r"Device %s RSSI: (-?\d+)" % BAND_MAC.replace(":", ":"))
        while True:
            try:
                p = subprocess.run(
                    ["timeout", "14", "bluetoothctl"],
                    input="scan on\nquit\n", capture_output=True,
                    text=True, timeout=20)
                m = pat.search(p.stdout)
                if m:
                    global _band_last_seen
                    _band_last_seen = time.time()
            except Exception:
                pass
            try:
                _wifi_station_check(log=log)
            except Exception:
                pass
            time.sleep(30)

    threading.Thread(target=_loop, daemon=True).start()
    log(f"[owner] BLE heartbeat started ({BAND_MAC})")


def band_recently_seen():
    return (time.time() - _band_last_seen) < BAND_FRESH_S


def wifi_recently_seen():
    return (time.time() - _wifi_last_seen) < WIFI_FRESH_S


def owner_nearby():
    """双路在场信号: 手环BLE 或 热点连接, 任一可见即在场"""
    return band_recently_seen() or wifi_recently_seen()


# ===== 单目标跟踪器 (α-β滤波, 方案④: 预测+门控关联, 纯算术零开销) =====
ALPHA = 0.7              # 位置修正系数
BETA = 0.35              # 速度修正系数
CONT_BONUS = 0.08        # 连续性加分满格值(距预测越近加越多)
NEAR_PX = 80.0           # 连续性加分满格距离
SNAP_GATE_PX = 220.0     # 硬门控: 距预测超过此值的候选重罚(防吸到别人身上)


class TargetTracker:
    """恒速模型预测目标下一帧位置, 给关联打分用"""

    def __init__(self):
        self.reset()

    def reset(self):
        self.x = self.y = None
        self.vx = self.vy = 0.0

    def predict(self):
        if self.x is None:
            return None
        return (self.x + self.vx, self.y + self.vy)

    def update(self, cx, cy):
        if self.x is None:
            self.x, self.y = cx, cy
            self.vx = self.vy = 0.0
            return
        px, py = self.x + self.vx, self.y + self.vy
        rx, ry = cx - px, cy - py
        self.x = px + ALPHA * rx
        self.y = py + ALPHA * ry
        self.vx += BETA * rx
        self.vy += BETA * ry

    def coast(self):
        """本帧没匹配到: 按原速度外推(供下帧预测), 速度不衰减太快"""
        if self.x is not None:
            self.vx *= 0.8
            self.vy *= 0.8
            self.x += self.vx
            self.y += self.vy


def _fetch_detections():
    """拉person检测列表。yolo_daemon偶发把坐标序列化成字符串 → 这里统一强转float

    ⚠️ 机主原则第0层: YOLO person 是唯一候选先决条件。
    本函数是全链路唯一的候选入口, 非person永远进不了打分池;
    模板/跟踪器/在场信号都只是在此之上的偏好与状态层, 任何改动不得放宽此处。"""
    try:
        req = urllib.request.Request(DET_API, headers={"User-Agent": "owner"})
        d = json.loads(urllib.request.urlopen(req, timeout=1.0).read())
    except Exception:
        return []
    out = []
    for x in d.get("detections", []):
        try:
            det = {"c": str(x.get("c")),
                   "x1": float(x["x1"]), "y1": float(x["y1"]),
                   "x2": float(x["x2"]), "y2": float(x["y2"]),
                   "p": float(x.get("p", 0.0))}
        except (KeyError, TypeError, ValueError):
            continue  # 单条坏数据直接丢弃, 不拖垮整帧
        if det["c"] != "person":
            continue
        # 幻影人过滤: YOLO常对椅子/包等以低置信度误报成person
        if det["p"] < CONF_MIN or (det["y2"] - det["y1"]) < MIN_BOX_H:
            continue
        out.append(det)
    return out


def _largest(dets):
    best = None
    for det in dets:
        area = (det["x2"] - det["x1"]) * (det["y2"] - det["y1"])
        if best is None or area > best["area"]:
            best = {"x1": det["x1"], "y1": det["y1"],
                    "x2": det["x2"], "y2": det["y2"], "area": area}
    return best


def _roi_hist(hsv, box):
    """bbox 内的 H-S 二维直方图, 归一化。低饱和像素(黑白灰衣服)自动权重低"""
    x1 = int(round(box["x1"]))
    y1 = int(round(box["y1"]))
    x2 = int(round(box["x2"]))
    y2 = int(round(box["y2"]))
    h, w = hsv.shape[:2]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w, x2), min(h, y2)
    if x2 - x1 < 8 or y2 - y1 < 8:
        return None
    roi = hsv[y1:y2, x1:x2]
    hist = cv2.calcHist([roi], [0, 1], None, [H_BINS, S_BINS], [0, 180, 0, 256])
    cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
    return hist


def _aspect(box):
    w = max(1e-6, box["x2"] - box["x1"])
    h = max(1e-6, box["y2"] - box["y1"])
    return w / h


class OwnerProfile:
    """主人外观模板: 平均 HSV 直方图 + 中位体态比 (+饱和度能量用于自适应权重)
    + 下装(腿带)直方图: 只服务"底盘转向弱线索", 不参与主人判定。"""

    def __init__(self, hist, aspect, samples, sat_energy=None, leg_hist=None):
        self.hist = hist
        self.aspect = aspect
        self.samples = samples
        self.sat_energy = sat_energy
        self.leg_hist = leg_hist      # 框下半带直方图(见 find_leg_band)

    def color_sim(self, hist):
        if self.hist is None or hist is None:
            return 0.5  # 没有可比信息时不惩罚(夜间/纯色墙等极端情况)
        d = cv2.compareHist(self.hist, hist, cv2.HISTCMP_BHATTACHARYYA)
        return max(0.0, 1.0 - float(d))

    def aspect_sim(self, aspect):
        diff = abs(aspect - self.aspect) / max(1e-6, self.aspect)
        return max(0.0, 1.0 - diff / 0.5)

    def score_box(self, hsv, box):
        w_c, w_a = W_COLOR, W_ASPECT
        # 低饱和模板(灰白黑衣服)颜色区分度差 → 自动降权, 靠体态+连续性
        if self.sat_energy is not None and self.sat_energy < SAT_LOW:
            w_c, w_a = 0.3, 0.7
        return w_c * self.color_sim(_roi_hist(hsv, box)) \
             + w_a * self.aspect_sim(_aspect(box))

    def find_leg_band(self, img_bgr):
        """画面下半带的"下装(腿)"颜色匹配 → (u_px, score) 或 None。

        用途: 主人近距擦身时画面里只剩腿(无头无躯干), YOLO 出不了 person → 用
        颜色反投影 + 连通域找出腿的横向位置, 给**底盘转向通道**一个方向权重,
        好让车继续转而不是"没有人框就停下"。

        ⚠️ 机主原则第0层(owner_id.py:157): 本函数不产生主人候选 —— 不进
        `_fetch_detections`/`select_target`、不写 owner.json、不进 KF、不刷新模板,
        也绝不放行前进(前进只由强证据授权)。
        """
        if self.leg_hist is None or img_bgr is None:
            return None
        try:
            hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
            h, w = hsv.shape[:2]
            band = hsv[int(h * LEG_BAND_TOP_FRAC):, :]
            # 直方图是 [0,1] 归一化的 float64 → 必须转 float32 并给 scale=255,
            # 否则 calcBackProject 输出只有 0/1 两级(实测), 二值化会把目标全滤掉
            hist_t = np.float32(self.leg_hist)
            bp = cv2.calcBackProject([band], [0, 1], hist_t, [0, 180, 0, 256], 255)
            bp = cv2.medianBlur(bp, 5)
            _, thr = cv2.threshold(bp, LEG_BP_THRESH, 255, cv2.THRESH_BINARY)
            n, _lab, stats, _cent = cv2.connectedComponentsWithStats(thr, 8)
            best = None
            for i in range(1, n):
                x, y, ww, hh, area = (int(stats[i][0]), int(stats[i][1]),
                                      int(stats[i][2]), int(stats[i][3]),
                                      int(stats[i][4]))
                if area < LEG_MIN_AREA or hh < 20:
                    continue
                if hh < LEG_MIN_ASPECT * ww:      # 腿是竖条: 横比竖宽的块多半是地面/墙裙
                    continue
                roi = band[y:y + hh, x:x + ww]
                hist = cv2.calcHist([roi], [0, 1], None, [H_BINS, S_BINS],
                                    [0, 180, 0, 256])
                cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
                sim = 1.0 - float(cv2.compareHist(self.leg_hist, hist,
                                                  cv2.HISTCMP_BHATTACHARYYA))
                score = sim * min(1.0, area / (2.0 * LEG_MIN_AREA))
                if best is None or score > best[1]:
                    best = (x + ww / 2.0, score)
            return best
        except Exception:
            return None


def read_recent_frame():
    """读双缓冲里较新的一帧(两帧都在 ~150ms 内, 任取其一即可)"""
    try:
        path = FRAME_A
        img = cv2.imread(path)
        if img is None:
            img = cv2.imread(FRAME_B)
        return img
    except Exception:
        return None


def enroll(log=print):
    """快速认主: 采样 ENROLL_DURATION_S 秒内最大 person, 返回 OwnerProfile 或 None"""
    hists, aspects, sats = [], [], []
    leg_hists = []           # 下装(腿带)模板: 只喂转向弱线索, 不参与主人判定
    t_end = time.time() + ENROLL_DURATION_S
    while time.time() < t_end:
        box = _largest(_fetch_detections())
        if box:
            img = read_recent_frame()
            if img is not None:
                hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                h = _roi_hist(hsv, box)
                if h is not None:
                    hists.append(h)
                    aspects.append(_aspect(box))
                    x1 = max(0, int(box["x1"])); y1 = max(0, int(box["y1"]))
                    x2 = min(hsv.shape[1], int(box["x2"]))
                    y2 = min(hsv.shape[0], int(box["y2"]))
                    if x2 - x1 > 8 and y2 - y1 > 8:
                        sats.append(float(hsv[y1:y2, x1:x2, 1].mean()))
                        # 下装 = 框下部 40%(擦身时只剩这一段在画面里)
                        leg = _roi_hist(hsv, {"x1": box["x1"],
                                              "y1": box["y1"] + (y2 - y1) * 0.6,
                                              "x2": box["x2"], "y2": box["y2"]})
                        if leg is not None:
                            leg_hists.append(leg)
        time.sleep(ENROLL_INTERVAL_S)
    log(f"[owner] enroll samples={len(hists)}")
    if len(hists) < 4:
        return None
    sat = float(np.mean(sats)) if sats else None
    mean_hist = np.mean(np.stack(hists), axis=0)
    cv2.normalize(mean_hist, mean_hist, 0, 1, cv2.NORM_MINMAX)
    leg_hist = None
    if len(leg_hists) >= 4:
        leg_hist = np.mean(np.stack(leg_hists), axis=0)
        cv2.normalize(leg_hist, leg_hist, 0, 1, cv2.NORM_MINMAX)
    log(f"[owner] template sat_energy={sat:.0f} leg_samples={len(leg_hists)}")
    return OwnerProfile(mean_hist, float(np.median(aspects)), len(hists), sat, leg_hist)


def select_target(profile, img_bgr, dets, tracker=None):
    """从检测列表中选出主人。
    profile 为 None 时降级返回最大 person。
    tracker: TargetTracker, 有预测时做 门控关联(连续性加分+远距重罚)
    返回 (box 或 None, 得分)"""
    if not dets:
        return None, 0.0
    if profile is None:
        return _largest(dets), 1.0
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    pred = tracker.predict() if tracker is not None else None
    best, best_s = None, -1.0
    for det in dets:
        box = {"x1": det["x1"], "y1": det["y1"], "x2": det["x2"], "y2": det["y2"]}
        cx, cy = (box["x1"] + box["x2"]) / 2, (box["y1"] + box["y2"]) / 2
        s = profile.score_box(hsv, box)
        if pred is not None:
            dist = ((cx - pred[0]) ** 2 + (cy - pred[1]) ** 2) ** 0.5
            s += CONT_BONUS * max(0.0, 1.0 - dist / NEAR_PX)
            if dist > SNAP_GATE_PX:
                s -= 0.15   # 软门控: 轻微抑制远距候选, 但不锁死(防错锁陷阱)
        if s > best_s:
            best, best_s = box, s
    if best_s < SCORE_MIN:
        return None, best_s
    return best, best_s


def _fetch_raw_persons():
    """原始 person 列表(**不做层0过滤**)。

    ⚠️ 机主原则第0层: 候选池入口仍是 `_fetch_detections`(全链路唯一), 本函数是并列的
    旁路, 只喂"转向弱线索"。"放宽门槛"与"旁路一条只驱动原地转的通道"是两件事——
    前者会让幻影重新进候选池(夜景 95/98 帧事故), 后者最坏只是多转几度。
    """
    try:
        req = urllib.request.Request(DET_API, headers={"User-Agent": "owner-weak"})
        d = json.loads(urllib.request.urlopen(req, timeout=1.0).read())
    except Exception:
        return []
    out = []
    for x in d.get("detections", []):
        try:
            out.append({"c": str(x.get("c")), "p": float(x.get("p", 0.0)),
                        "x1": float(x["x1"]), "y1": float(x["y1"]),
                        "x2": float(x["x2"]), "y2": float(x["y2"])})
        except (KeyError, TypeError, ValueError):
            continue
    return out


def weak_cue(profile=None, dets=None, img_bgr=None):
    """底盘转向弱线索: 返回 {"dir","score","src"} 或 None。

    只回答"往哪边转"(dir 与 follow 的 offset 同号约定: 图右=R), 不回答"谁是主人"。
    来源(取分高者):
      ① low_conf: 被层0拦下的低置信/贴底框 —— 零 CV 成本, 通常最先命中
      ② leg_band: 认主时派生的"下装模板" + 画面下半带颜色反投影/CamShift 类匹配
                  (主人近距擦身时画面里只剩腿, YOLO 出不了 person)

    ⚠️ 绝不: 写 owner.json / 进 select_target / 进 KF / 刷新模板 / 放行前进。
    """
    cands = []
    W = 640.0
    try:
        raw = _fetch_raw_persons() if dets is None else dets
        for d in raw:
            if str(d.get("c")) != "person":
                continue
            p = float(d.get("p", 0.0))
            bh = float(d["y2"]) - float(d["y1"])
            if p < LOWCONF_MIN_P:
                continue
            if p >= CONF_MIN and bh >= MIN_BOX_H:
                continue                     # 层0已收 → 走强通道, 不重复
            clipped = float(d["y2"]) >= 478.0 or float(d["y1"]) <= 2.0   # 贴底/贴顶(半身出画)
            score = min(1.0, p / CONF_MIN) * (1.0 if clipped else 0.75)
            u = (float(d["x1"]) + float(d["x2"])) / 2.0
            cands.append({"dir": "R" if u > W / 2 else "L",
                          "score": round(score, 3), "src": "low_conf"})
    except Exception:
        pass
    try:
        if profile is not None:
            img = img_bgr if img_bgr is not None else read_recent_frame()
            got = profile.find_leg_band(img) if img is not None else None
            if got is not None:
                u, score = got
                cands.append({"dir": "R" if u > W / 2 else "L",
                              "score": round(float(score), 3), "src": "leg_band"})
    except Exception:
        pass
    if not cands:
        return None
    return max(cands, key=lambda c: c["score"])
