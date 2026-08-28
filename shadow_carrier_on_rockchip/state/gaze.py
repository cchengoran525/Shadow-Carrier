#!/usr/bin/env python3
"""gaze.py v2 - 视野保持器 (机主定稿的级联式方案)

设计哲学(2026-08-28 机主拍板):
  云台只做一件事——人快出画面时缓缓转过去看住, 平时不动。
  底盘由 follow_controller 原生逻辑驱动(前进接近+bbox转向)。
  稳态: 底盘朝向逐渐对准人 → 人自动回画面中央 → 云台自然回正。
  (级联PID: 云台=内环视野保持, 底盘=外环航向追踪)

状态:
  KEEP    人可见且在安全区(中央±KEEP_EDGE)   → 纹丝不动
  KEEP_TURN 人可见但靠近/越过边缘            → 缓转看住(KEEP_RATE)
  HOLD    丢检 <1.5s                         → 冻结
  SEARCH  丢检 1.5~4s                        → 最后位置±SCAN_AMP慢扫
  CENTER  丢检 >4s                           → 归中(PAN_FORWARD)

符号说明: 固件 PAN_INVERT=true 下, 实测约定
  bearing<0 = 目标物理左侧 → 修正是 pan 应减小 (pan += sign(bearing)*rate*dt)
  若固件关闭 INVERT, 把 PAN_SIGN 改为 +1。
"""
import math

from calib.params import CX, FX, K1, K2

# ================= 常数 =================
PAN_FORWARD = 90.58        # 光轴正前方指令角
PAN_SIGN = -1              # +1=指令增大向左; -1=指令增大向右(PAN_INVERT=true)
KEEP_EDGE_DEG = 24.0       # 救援阈值: 人到画面24°(接近出画)才出手 (12°太敏感会持续扰动follow)
KEEP_RATE = 10.0           # 救援缓转速度 (度/s)
HOLD_TIMEOUT = 1.5
SEARCH_TIMEOUT = 4.0
SCAN_AMP = 45.0            # 定向扫描最大扩展角(度)
SCAN_RATE = 20.0
CONF_MIN = 0.5
# ========================================


def _wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def u_to_bearing(u_px):
    """图像u → 相对光轴方位角偏移(度, 含去畸变)。符号随镜像: 见 PAN_SIGN"""
    u_n = (u_px - CX) / FX
    ud = u_n
    for _ in range(8):
        r2 = ud * ud
        ud = u_n / (1 + K1 * r2 + K2 * r2 * r2)
    return math.degrees(math.atan(ud))


class GazeController:
    def __init__(self, pan_now):
        self.mode = "CENTER"
        self.pan = pan_now
        self.pan_last_seen = pan_now
        self.last_seen_t = None
        self.last_exit_sign = 1     # 人消失时在光轴哪侧: +1右/-1左 → 定向扫描依据
        self._out = {"pan_deg": pan_now, "source": "CENTER", "t": 0.0}

    def feed(self, t, owner_u):
        if owner_u is not None:
            bearing = u_to_bearing(owner_u)
            if abs(bearing) > 0.5:
                self.last_exit_sign = 1 if bearing > 0 else -1
            if abs(bearing) > KEEP_EDGE_DEG:
                self.mode = "KEEP_TURN"
                # 向边缘方向缓转看住: bearing正(左) × PAN_SIGN(-1) → pan减小
                self.pan = _wrap180(
                    self.pan + PAN_SIGN * (bearing / abs(bearing)) * KEEP_RATE * 0.1)
            else:
                self.mode = "KEEP"
            self.pan_last_seen = self.pan
            self.last_seen_t = t
            self._emit(t)
            return self._out

        # ---- 丢检 ----
        loss = t - (self.last_seen_t if self.last_seen_t is not None else t)
        if loss < HOLD_TIMEOUT:
            self.mode = "HOLD"                       # 冻结
        elif loss < SEARCH_TIMEOUT:
            self.mode = "SEARCH"
            # 定向扩展扫描: 朝人消失的那一侧越扫越远 (10°→45°), 不再无方向正弦摆
            sweep_dir = PAN_SIGN * self.last_exit_sign
            offset = min(SCAN_RATE * (loss - HOLD_TIMEOUT), SCAN_AMP)
            self.pan = _wrap180(self.pan_last_seen + sweep_dir * offset)
        else:
            self.mode = "CENTER"
            delta = _wrap180(PAN_FORWARD - self.pan)
            if abs(delta) > 1.0:
                self.pan = _wrap180(self.pan + max(-1.0, min(1.0, delta)))
        self._emit(t)
        return self._out

    def _emit(self, t):
        self._out = {"pan_deg": round(self.pan, 2),
                     "source": self.mode, "t": t}


if __name__ == "__main__":
    # 闭环演示: 主人固定在物理左21°, 前3s可见(验证边缘保持,不主动居中), 后5s消失
    import time
    THETA_TRUE = 21.0

    def observe(pan):
        # 真实摄像头为镜像画面: 物理左(off+) → u减小 (与瓶子实测一致)
        off = _wrap180(THETA_TRUE - (pan - PAN_FORWARD))
        return CX - FX * math.tan(math.radians(off))

    g = GazeController(90.58)
    t = 0.0
    while t < 8.0:
        u = observe(g.pan) if t <= 3.0 else None
        out = g.feed(t, u)
        if round(t * 10) % 5 == 0:
            off = _wrap180(THETA_TRUE - (g.pan - PAN_FORWARD))
            print(f"t={t:.1f} mode={g.mode:9s} pan={g.pan:7.2f} off={off:+6.2f}°")
        t += 0.1
        time.sleep(0.02)
