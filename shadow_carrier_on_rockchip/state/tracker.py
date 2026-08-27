#!/usr/bin/env python3
"""tracker.py - 极坐标匀速CV卡尔曼滤波 (单目标)

状态 x = [θ, θ̇, r, ṙ]   (车体系极坐标: 方位角deg / 距离m)
量测 z = [θ_meas, r_meas] (decoder输出, 带各自方差)

设计要点:
- 所有方法显式传入时间戳 t (单调钟秒), 不读墙钟 → 假数据/实机同一套代码
- update 支持乱序/延迟: 按量测自身时间戳更新后, 由调用方 predict_to(now) 取当前估计
- 新息卡方软门限: 轻度离群自动膨胀R降权; 严重离群(d²>25)判定重捕获, 硬复位
- quality = exp(-age/1.2): 距上次量测的时间衰减, 供凝视状态机/下游消费
"""
import math

import numpy as np

DEG = math.pi / 180.0


def _wrap180(a):
    """角度归一化到 (-180,180]"""
    return (a + 180.0) % 360.0 - 180.0


class PolarCVKF:
    # 噪声模型 (初版草案, 实机按 decoder 误差模型调)
    SIG_TH_MEAS = 2.0     # 方位量测噪声 deg
    SIG_A_TH = 8.0        # 方位过程噪声(加速度谱) deg/s²  降Q→更信模型,平滑更强
    SIG_A_R = 1.2         # 距离过程噪声 m/s²
    GATE_SOFT = 9.0       # 卡方软门限 (2维, ~99%)
    GATE_HARD = 16.0      # 硬门限 → 视为重捕获

    def __init__(self, t0, theta0, r0):
        self.x = np.array([theta0, 0.0, r0, 0.0], float)
        self.P = np.diag([25.0, 100.0, 0.5, 2.0])   # 初始不确定度大
        self.t = float(t0)
        self.quality = 1.0
        self.coasting = False
        self.nis_last = 0.0

    # ---------- 内部 ----------
    def _F_Q(self, dt):
        F = np.eye(4)
        F[0, 1] = dt
        F[2, 3] = dt
        # 分段白噪声加速度模型
        G = np.array([[0.5 * dt * dt, 0], [dt, 0], [0, 0.5 * dt * dt], [0, dt]])
        Q = np.outer(G[:, 0], G[:, 0]) * self.SIG_A_TH ** 2 + \
            np.outer(G[:, 1], G[:, 1]) * self.SIG_A_R ** 2
        return F, Q

    def _H_R(self, r_est):
        R = np.diag([self.SIG_TH_MEAS ** 2,
                     (0.15 + 0.06 * abs(r_est)) ** 2])   # 距离方差随距离增长
        return np.array([[1, 0, 0, 0], [0, 0, 1, 0]], float), R

    # ---------- 对外 ----------
    def predict_to(self, t):
        """把状态外推到时刻 t (不改变 last_update 的quality语义)"""
        dt = t - self.t
        if dt <= 0:
            return
        F, Q = self._F_Q(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.x[0] = _wrap180(self.x[0])
        self.t = float(t)

    def update(self, t, theta_meas, r_meas, r_var_scale=1.0):
        """按量测时间戳融合; 返回归一化新息 d² (诊断用)
        门限策略按间隔分治:
          间隔>1s(久别重逢): d²超硬门限 → 硬复位到量测(目标可能真跳了)
          间隔≤1s(连续跟踪): d²超硬门限 → 整帧拒绝(极端离群,如误检)"""
        age = t - self.t
        self.predict_to(t)
        H, R = self._H_R(self.x[2])
        R = R * max(r_var_scale, 0.1)

        z = np.array([_wrap180(theta_meas), r_meas])
        y = z - H @ self.x
        y[0] = _wrap180(y[0])
        S = H @ self.P @ H.T + R
        d2 = float(y @ np.linalg.solve(S, y))
        self.nis_last = d2

        if d2 > self.GATE_HARD:
            if age > 1.0:
                # 重捕获: 硬复位位置, 速度清零, 方差放大
                self.x[0] = _wrap180(theta_meas)
                self.x[2] = r_meas
                self.x[1] = self.x[3] = 0.0
                self.P = np.diag([25.0, 100.0, 1.0, 2.0])
            else:
                # 连续跟踪中的极端离群(误检): 整帧拒绝, 状态保持预测值
                self.t = float(t)
                self.coasting = False
                return d2
        else:
            if d2 > self.GATE_SOFT:
                R = R * (d2 / self.GATE_SOFT)      # 软降权
                S = H @ self.P @ H.T + R
            K = self.P @ H.T @ np.linalg.inv(S)
            self.x = self.x + K @ y
            self.x[0] = _wrap180(self.x[0])
            I_KH = np.eye(4) - K @ H
            self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T   # Joseph form

        self.t = float(t)
        self.coasting = False
        return d2

    def state(self, t_now):
        """外推到 t_now 并返回估计快照 (不消耗滤波器内部时间基准)"""
        dt = t_now - self.t
        F, _ = self._F_Q(max(dt, 0))
        xt = F @ self.x
        xt[0] = _wrap180(xt[0])
        age = max(dt, 0)
        self.quality = math.exp(-age / 1.2)
        self.coasting = age > 0.45     # 超过一个常规检测间隔视为盲推
        return {"t": t_now, "theta": float(xt[0]), "theta_dot": float(xt[1]),
                "r": float(xt[2]), "r_dot": float(xt[3]),
                "quality": round(self.quality, 3), "coasting": self.coasting,
                "age_s": round(age, 3), "nis": round(self.nis_last, 2)}


if __name__ == "__main__":
    # 迷你演示: 1Hz量测, 每0.2s查询当前估计
    import time
    kfx = PolarCVKF(0.0, -20.0, 2.5)
    t = 0.0
    for i in range(6):
        t += 1.0
        kfx.update(t, -20.0 + i * 2.0, 2.5 - i * 0.1)
        print(f"t={t:.1f}s  {kfx.state(t + 0.2)}")
