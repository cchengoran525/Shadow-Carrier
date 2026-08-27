#!/usr/bin/env python3
"""solve_geom.py - 用桌面实测点反解等效焦距与几何偏移
物理模型:
    设光心在胶带坐标系下的横向偏移 dx_cm、纵向距离偏移 dd_cm
    则第 i 点: atan((L_i+dx) / (D_i+dd)) ≈ 解码方位角的逆
    但解码角本身依赖 (fx,cx)。这里不标 cx/fx, 直接拟合:
        u_px = A + B * tan(true_angle)
    其中 B≈fx(考虑畸变近似线段中段), A≈cx+dx比例项。
    拟合质量 r^2 报告; 若残差系统性弯曲则提示畸变区参与过多。
输入: points.json  [{"L":cm,"D":cm,"u":px}, ...]
"""
import json
import math
import sys

import numpy as np


def main(path):
    pts = json.load(open(path))
    L = np.array([p["L"] for p in pts], float)
    D = np.array([p["D"] for p in pts], float)
    U = np.array([p["u"] for p in pts], float)

    # 初值: A=cx(310), B=fx(757); 用最小二乘迭代解含 dd/dx 的非线性模型
    # 参数向量 p=[A, B, dx, dd]
    def predict(p):
        A, B, dx, dd = p
        return A + B * ((L + dx) / (D + dd))

    p = np.array([310.0, 700.0, 0.0, 0.0])
    for it in range(400):
        pred = predict(p)
        r = U - pred
        J = np.zeros((len(L), 4))
        eps = 1e-4
        for k in range(4):
            q = p.copy()
            q[k] += eps
            J[:, k] = (predict(q) - pred) / eps
        dp, *_ = np.linalg.lstsq(J, r, rcond=None)
        p += dp
        if np.abs(dp).max() < 1e-6:
            break

    A, B, dx, dd = p
    resid = U - predict(p)
    ss_res = float(np.sum(resid ** 2))
    ss_tot = float(np.sum((U - U.mean()) ** 2))
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 1.0

    print(f"迭代{it+1}次收敛")
    print(f"A (等效cx+常数)   = {A:.1f} px")
    print(f"B (等效fx)        = {B:.1f} px")
    print(f"dx (光心横偏)     = {dx:+.1f} cm")
    print(f"dd (光心纵偏)     = {dd:+.1f} cm")
    print(f"r²                = {r2:.4f}")
    print("\n各点残差(px):")
    for pt, rr in zip(pts, resid):
        print(f"  L={pt['L']:+.0f} D={pt['D']:.0f}: u={pt['u']:.1f} "
              f"pred={float(A+B*((pt['L']+dx)/(pt['D']+dd))):.1f} "
              f"resid={rr:+.2f}")

    out = {"A_cx_eff": round(float(A), 2), "B_fx_eff": round(float(B), 2),
           "dx_cm": round(float(dx), 2), "dd_cm": round(float(dd), 2),
           "r2": round(r2, 5)}
    json.dump(out, open("solved_geom.json", "w"), indent=1)
    print("\n已存 solved_geom.json")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "points.json"))
