#!/usr/bin/env python3
"""
selftest_polar.py —— polar_mapper 数学自检（合成相机, 无需硬件）
方法: 虚拟相机(h=0.10m, 已知俯角) 生成地面网格点 -> 正投影到像素 ->
      用 polar_mapper 逆映射回极坐标 -> 检查往返误差
通过标准: 方位角误差 < 0.5°, 距离误差 < 2%(近环) / 5%(远环)
"""
import math
import polar_mapper as pm

FX, FY, CX, CY = pm.FX, pm.FY, pm.CX, pm.CY


def project(x_fwd, y_right, h_cam, tilt_down_deg):
    """地面点(车体系: x前 y右) -> 像素(u,v). 相机在原点上方 h_cam, 俯角 tilt_down"""
    # 相机坐标系: z光轴向前, y向下
    t = math.radians(tilt_down_deg)
    # 车体系 -> 相机系(先平移到相机位置, 再绕右轴转俯角)
    dx, dy, dz = x_fwd, y_right, -h_cam          # 点在相机坐标(未旋转)
    # 俯角向下 tilt: 绕 y(右) 轴旋转
    xc = dx * math.cos(t) - dz * math.sin(t)     # 沿光轴方向分量
    yc = dy
    zc = -(dx * math.sin(t) + dz * math.cos(t))  # 向下分量 -> 像素v方向
    if xc <= 0:
        return None  # 在相机背后
    u = FX * yc / xc + CX
    v = FY * zc / xc + CY
    return u, v


def main():
    print("虚拟相机: h=0.10m, tilt_down=17.5° (cmd=%.0f)" % (112 + 17.5))
    tilt_down = 17.5
    tilt_cmd = pm.TILT_LEVEL_CMD + tilt_down
    print("像素自检点: 地面 0.5/1.0/2.0/3.0m × 方向 -60/0/60°\n")
    print("%-14s %-12s %-14s %-10s %-10s" % ("真值(方位,距)", "像素(u,v)", "逆映射(方位,距)", "方位误差", "距离误差"))
    worst_b, worst_d = 0.0, 0.0
    for dist_true in (0.5, 1.0, 2.0, 3.0):
        for bear_true in (-60, 0, 60):
            x = dist_true * math.cos(math.radians(bear_true))
            y = dist_true * math.sin(math.radians(bear_true))
            uv = project(x, y, pm.H_CAM, tilt_down)
            if uv is None:
                print("(%6.1f°,%4.1fm) 落在相机背后, 跳过" % (bear_true, dist_true))
                continue
            u, v = uv
            b2, d2 = pm.pixel_to_polar(u, v, tilt_cmd)
            if b2 is None:
                # 俯角低于 PHI_MIN 被拒判: 检查该点确实超出可信范围
                phi = math.degrees(math.atan(pm.H_CAM / dist_true))
                ok = phi < pm.PHI_MIN_DEG
                print("(%6.1f°,%4.1fm) (%6.1f,%6.1f) 拒判(俯角%.1f°<%.0f°) %s"
                      % (bear_true, dist_true, u, v, phi, pm.PHI_MIN_DEG,
                         "OK(应拒)" if ok else "FAIL(不该拒)"))
                continue
            eb = abs(b2 - bear_true)
            ed = abs(d2 - dist_true) / dist_true * 100
            worst_b, worst_d = max(worst_b, eb), max(worst_d, ed)
            flag = "OK" if (eb < 0.5 and ed < (2 if dist_true <= 1.8 else 5)) else "FAIL"
            print("(%6.1f°,%4.1fm) (%6.1f,%6.1f) (%6.1f°,%4.2fm) %7.3f° %8.2f%%  %s"
                  % (bear_true, dist_true, u, v, b2, d2, eb, ed, flag))
    print("\nworst: bearing %.3f°  dist %.2f%%" % (worst_b, worst_d))
    # 扇区自检
    print("\n扇区函数自检: bear 0->sec0, 45->sec1, -45->sec7, 315->sec7")
    for b, want in ((0, 0), (45, 1), (-45, 7), (315, 7), (22, 0), (23, 1)):
        s, r = pm.polar_to_sector(b, 1.0)
        print("  bear %4d -> sector %d %s" % (b, s, "OK" if s == want else "FAIL want %d" % want))


if __name__ == "__main__":
    main()
