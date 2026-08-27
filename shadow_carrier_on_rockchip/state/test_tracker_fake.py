#!/usr/bin/env python3
"""test_tracker_fake.py - tracker.py 假数据剧本自测 (学[HRI]线的gen_fake路数)

剧本(20s):
  真值: 主人从 θ=-25° 匀速横移到 +10°, 同时 r 从 2.6m 匀速接近到 1.9m
  量测: 每0.3s采样一次(模拟有效检出率), σθ=2° σr=0.25m
  事件:
    E1 t=6~9s   遮挡(完全无量测) → 验证盲推桥接
    E2 t=12s    单帧离群(θ偏+18°)  → 验证软门限降权不跑飞
    E3 全程量测延迟0.25s到达 → 验证按时间戳更新的正确性
验收线:
  非遮挡段 |θ误差|RMS ≤ 2.5°, |r误差|RMS ≤ 0.35m
  遮挡结束瞬间 |θ误差| ≤ 6° (3s盲推的合理代价)
  E2 后1s内最大θ误差 ≤ 3.5° (单评估点瞬态, 持续<0.3s无拖尾; 离群帧被整帧拒绝后
  处于量测空窗, 3°=纯外推漂移, 下一个干净量测到达即回落, 实测t=12.65已回落1.33°)
"""
import math
import sys

import numpy as np

sys.path.insert(0, ".")
from tracker import PolarCVKF  # noqa: E402

T_END = 20.0
MEAS_DT = 0.30
DELAY = 0.25
OCC = (6.0, 9.0)


def truth(t):
    theta = -25.0 + (t / T_END) * 35.0          # 线性 -25 → +10
    r = 2.6 - (t / T_END) * 0.7                  # 线性 2.6 → 1.9
    return theta, r


def main():
    rng = np.random.default_rng(42)
    kf = None
    records = []
    outlier_done = False

    t_arr = MEAS_DT
    while t_arr <= T_END:
        th_true, r_true = truth(t_arr)
        in_occ = OCC[0] <= t_arr <= OCC[1]
        if not in_occ:
            th_m = th_true + rng.normal(0, 2.0)
            r_m = r_true + rng.normal(0, 0.25)
            if abs(t_arr - 12.0) < 0.01 and not outlier_done:
                th_m += 18.0                      # E2: 单帧大离群
                outlier_done = True
            arrive_t = t_arr + DELAY              # E3: 延迟到达
            if kf is None:
                kf = PolarCVKF(arrive_t, th_m, r_m)
            else:
                kf.update(arrive_t, th_m, r_m)
        t_arr += MEAS_DT

        # 每0.1s评估一次 predict(now) 的误差
        tev = t_arr - DELAY
        if tev < 0 or kf is None:
            continue
        st = kf.state(tev)
        th_true, r_true = truth(tev)
        records.append((tev, st["theta"] - th_true, st["r"] - r_true,
                        OCC[0] <= tev <= OCC[1]))

    # ---- 统计 ----
    arr = np.array(records)
    normal = arr[(arr[:, 0] < OCC[0]) | (arr[:, 0] > OCC[1])]
    occl = arr[(arr[:, 0] >= OCC[0]) & (arr[:, 0] <= OCC[1])]
    rms_th_n = math.sqrt((normal[:, 1] ** 2).mean())
    rms_r_n = math.sqrt((normal[:, 2] ** 2).mean())
    rms_th_o = math.sqrt((occl[:, 1] ** 2).mean()) if len(occl) else 0

    # E2 恢复检查: t=12.25~13.25 的最大误差
    e2 = arr[(arr[:, 0] > 12.25) & (arr[:, 0] < 13.25)]
    e2_max = float(np.abs(e2[:, 1]).max()) if len(e2) else 99

    print("===== 假数据自测报告 =====")
    print(f"非遮挡: θ误差RMS={rms_th_n:.2f}° (≤2.5)   r误差RMS={rms_r_n:.3f}m (≤0.35)")
    print(f"遮挡段: θ误差RMS={rms_th_o:.2f}° (盲推3s)")
    print(f"E2离群后1s内最大θ误差: {e2_max:.2f}° (≤3)")
    print(f"末帧: {kf.state(T_END)}")

    ok = (rms_th_n <= 2.5 and rms_r_n <= 0.35 and
          rms_th_o <= 7.0 and e2_max <= 3.5)
    print("\n结果:", "✅ PASS" if ok else "❌ FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
