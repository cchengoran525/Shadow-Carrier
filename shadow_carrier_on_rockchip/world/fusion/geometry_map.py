#!/usr/bin/env python3
"""
geometry_map.py —— 滚动地图（P0-② HIDE 数据供给）
思路: FOLLOW 途中被动累积门/障碍/自由扇区/物体观测, 持久化到 world_map.json;
     HIDE 瞬间直接读最新地图(无需转云台); 地图空时才走扫掠兜底。

关键设计:
  - 每个条目带 `last_seen` 时间戳 + `hits` 计数; 超过 TTL 自动遗忘(避免陈旧误导)
  - 同目标关联: 扇区相同 + 方位角/距离在容差内 -> EWMA 更新, hits++
  - 距离有效性: 需与实际工作俯仰角匹配的 PnP 标定; 未标定时标记 dist_valid=false
  - 输出供 HRI: doors[] 带 age_s; free_sectors 带 age_s
用法(库): 
    m = RollingMap.load(path); m.update(snapshot); m.save(path)
"""
import json
import math
import os
import time

TTL_S = 30.0            # 条目存活时间
DOOR_BEAR_TOL = 20.0    # 门关联: 方位角容差(度)
DOOR_DIST_TOL = 0.6     # 门关联: 距离容差(m)
EWMA = 0.4              # 新观测权重
SECTOR_DEG = 45.0


def _sector(bearing):
    return int(((bearing + SECTOR_DEG / 2) % 360) // SECTOR_DEG) % 8


class RollingMap:
    def __init__(self, data=None):
        d = data or {}
        self.doors = d.get("doors", [])
        self.objects = d.get("objects", [])
        self.free_sectors = d.get("free_sectors", {})
        self.ts_updated = d.get("ts_updated", 0)
        self.frames = d.get("frames", 0)
        self.pose_calib_tilt_cmd = d.get("pose_calib_tilt_cmd", None)
        self.dist_valid = d.get("dist_valid", False)

    # ---------- 持久化 ----------
    @classmethod
    def load(cls, path):
        if os.path.exists(path):
            try:
                with open(path) as f:
                    return cls(json.load(f))
            except Exception:
                pass
        return cls()

    def save(self, path):
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.to_dict(), f, ensure_ascii=False)
        os.replace(tmp, path)

    def to_dict(self):
        return {"ts_updated": self.ts_updated, "frames": self.frames,
                "pose_calib_tilt_cmd": self.pose_calib_tilt_cmd,
                "dist_valid": self.dist_valid,
                "doors": self.doors, "objects": self.objects,
                "free_sectors": self.free_sectors}

    # ---------- 更新 ----------
    def update(self, snap):
        now = time.time()
        self.frames += 1
        self.ts_updated = now
        pose = snap.get("pose", {})
        self.pose_calib_tilt_cmd = pose.get("tilt_cmd")
        self.dist_valid = bool(pose.get("dist_calibrated", False))

        # 门: 关联或新增
        for d in snap.get("objects", []):
            if d.get("cls") != "door":
                continue
            b, dist = d.get("bearing_deg"), d.get("dist_m")
            if b is None:
                continue
            hit = None
            for e in self.doors:
                if abs(((e["bearing_deg"] - b + 180) % 360) - 180) <= DOOR_BEAR_TOL \
                   and (dist is None or e.get("dist_m") is None
                        or abs(e["dist_m"] - dist) <= DOOR_DIST_TOL):
                    hit = e
                    break
            if hit:
                hit["bearing_deg"] = round((1 - EWMA) * hit["bearing_deg"] + EWMA * b, 1)
                if dist is not None and hit.get("dist_m") is not None:
                    hit["dist_m"] = round((1 - EWMA) * hit["dist_m"] + EWMA * dist, 2)
                elif dist is not None:
                    hit["dist_m"] = dist
                hit["last_seen"] = now
                hit["hits"] += 1
                hit["sector"] = _sector(hit["bearing_deg"])
            else:
                self.doors.append({"bearing_deg": round(b, 1), "dist_m": dist,
                                   "sector": _sector(b), "ring": d.get("ring"),
                                   "last_seen": now, "hits": 1,
                                   "conf": d.get("conf")})

        # 物体(非门): 同样累积, 供"主人在饮水机旁"类语义
        for o in snap.get("objects", []):
            cls = o.get("cls")
            if cls == "door" or o.get("bearing_deg") is None:
                continue
            hit = None
            for e in self.objects:
                if e["cls"] == cls and abs(((e["bearing_deg"] - o["bearing_deg"] + 180) % 360) - 180) <= DOOR_BEAR_TOL:
                    hit = e
                    break
            if hit:
                hit["bearing_deg"] = round((1 - EWMA) * hit["bearing_deg"] + EWMA * o["bearing_deg"], 1)
                if o.get("dist_m") is not None:
                    hit["dist_m"] = o["dist_m"]
                hit["last_seen"] = now
                hit["hits"] += 1
            else:
                self.objects.append({"cls": cls, "bearing_deg": o.get("bearing_deg"),
                                     "dist_m": o.get("dist_m"), "last_seen": now, "hits": 1})

        # 自由扇区(近环)
        for s in snap.get("free_directions_deg", []):
            key = str(int(s / SECTOR_DEG) % 8)
            self.free_sectors[key] = now

        self._prune(now)
        return self

    def _prune(self, now):
        self.doors = [d for d in self.doors if now - d["last_seen"] <= TTL_S]
        self.objects = [o for o in self.objects if now - o["last_seen"] <= TTL_S]
        self.free_sectors = {k: t for k, t in self.free_sectors.items() if now - t <= TTL_S}

    # ---------- 供 HIDE 消费 ----------
    def summary(self, now=None):
        now = now or time.time()
        doors = []
        for d in self.doors:
            doors.append({"bearing_deg": d["bearing_deg"], "dist_m": d["dist_m"],
                          "sector": d["sector"], "hits": d["hits"],
                          "age_s": round(now - d["last_seen"], 1)})
        doors.sort(key=lambda x: x["age_s"])
        free = [{"sector": int(k), "bearing_deg": int(k) * 45,
                 "age_s": round(now - t, 1)} for k, t in self.free_sectors.items()]
        free.sort(key=lambda x: x["age_s"])
        return {"doors": doors, "free_sectors": free,
                "frames": self.frames, "dist_valid": self.dist_valid,
                "pose_calib_tilt_cmd": self.pose_calib_tilt_cmd,
                "age_s": round(now - self.ts_updated, 1) if self.ts_updated else None,
                "empty": len(doors) == 0 and len(free) == 0}
