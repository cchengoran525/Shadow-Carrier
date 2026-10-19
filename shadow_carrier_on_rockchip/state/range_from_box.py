#!/usr/bin/env python3
"""range_from_box.py — 单目地面几何测距 (bbox 底边 → 人-车地面距离)

为什么需要它 (2026-10-19 审查):
  `follow_controller` 的近距保护原来只有两个信号 —— 真实距离 `dist_fn`(未接线) 与
  **框高兜底**(`STOP_BBOX_H=400`)。框高在近距会**饱和**: 65cm 机位 + 1.66m 人,
  ~2.2m 以内头顶/脚底都已出画, bbox 高度一路被镜框裁到满高 → 1m 和 2m 的框高一样。
  于是"很近但不在正前"的人既不停也不减速(见 NEAR_GATE_OFFSET 的注释), 而 BASE_SPD
  上调后制动距离 ≈0.9m → 这是 demo 擦身镜头的真实风险。

  正解不是调阈值, 而是换**几何**: 用 bbox 底边(脚)在画面里的行号 + 机位高度 + 光轴俯仰,
  由地面平面交点直接解出距离。它在任意方位角都成立, 与框高无关。

几何 (针孔, 光轴俯仰 tilt):
  y_b   : bbox 底边像素行(脚)
  fy,cy : 相机内参 (state/calib/params.py)
  h     : 镜头离地高度 (m)
  tilt  : 光轴相对水平面的仰角(度, 正=抬头). 跟随态云台冻结在 TILT_LEVEL_CMD(水平) → 0

  射线相对光轴的俯角:  a = atan((y_b - cy) / fy)     (y_b > cy 才在光轴下方=朝下看地面)
  相对水平面的俯角:    d = a - tilt
  地面交点距离:        D = h / tan(d)      (d<=0: 射线在地平线以上/水平 → 无交点)

约定/边界:
  - y_b <= cy 且 tilt==0 → d<=0 → 返回 None (脚在光轴以上/刚到地平线, 无地面交点)
  - 底边贴住画面底边(被裁) → 返回 None (⚠️ 关键边界: 65cm 机位 + 水平光轴时,
    画面底边对应 ≈1.58m —— **更近的地面根本不在画面里**, 所以近于 ~1.6m 的人没有
    "脚"可用, 本函数必然返回 None。近距(<1.6m)的保护只能靠: ① 正前锥的框高兜底
    ② follow 的速度门(方位角越大越慢) ③ 雷达(range_m, 任意方位)。
    本函数的有效区间是 **≈1.6m–6m**(对应 y ∈ [330, 479]), 用来替代"框高测距"做距离匹配/减速。
  - 与 [世界] PnP 测距(≤2%) 是两条独立通道: 本条只用于跟随的距离环, 别拿它替换标定链
"""
import math

DEFAULT_CAM_H_M = 0.65      # 机位高度 (V2 §15.6 定案 60–70cm, 取 65)
DEFAULT_FY = 492.70         # state/calib/params.py
DEFAULT_CY = 276.60
DEFAULT_TILT_DEG = 0.0      # 光轴水平 (TILT_LEVEL_CMD=112 实测)


def ground_distance_m(y_bottom_px, cam_h_m=DEFAULT_CAM_H_M, fy=DEFAULT_FY,
                      cy=DEFAULT_CY, tilt_deg=DEFAULT_TILT_DEG, frame_h=None):
    """bbox 底边行号 → 人-车地面距离(米)。不可解/不可信时返回 None。

    frame_h: 画面高(用于判定"底边被镜框裁掉"); None=不做裁切判定
    """
    try:
        y_b = float(y_bottom_px)
    except (TypeError, ValueError):
        return None
    if cam_h_m <= 0 or fy <= 0:
        return None
    a = math.atan2(y_b - cy, fy)                 # 相对光轴的俯角(向下为正)
    d = a - math.radians(float(tilt_deg))        # 相对水平面的俯角
    if d <= 1e-4:                                # 射线水平或朝上 → 与地面无交点
        return None
    dist = cam_h_m / math.tan(d)
    if frame_h is not None and y_b >= float(frame_h) - 1.0:
        # 底边贴着画面底边: 脚已出画, 底边不是"脚" → 距离只是下界
        return None
    if dist <= 0.1 or dist > 30.0:               # 物理上不可信
        return None
    return round(dist, 3)


if __name__ == "__main__":
    # 迷你自检: 已知几何 → 距离, 并验证单调性(画面越往下=y 越大 = 越近)
    print("y_b  → 距离(m)   [h=0.65, fy=492.7, cy=276.6, tilt=0]")
    prev = None
    for y in (300, 330, 360, 400, 440, 470):
        d = ground_distance_m(y)
        print(f"  {y:4d} → {d}")
        if d is not None and prev is not None:
            assert d < prev, "画面越往下(y 越大)应越近(单调性)"
        prev = d if d is not None else prev
    assert ground_distance_m(276.6) is None, "脚在光轴/地平线上 → 无交点"
    assert ground_distance_m(200.0) is None, "脚在光轴上方 → 无交点"
    assert ground_distance_m(479.0, frame_h=480) is None, "底边贴框 → 不可信"
    print("自检通过")
