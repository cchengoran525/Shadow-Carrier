#ifndef MECANUM_KINEMATICS_H
#define MECANUM_KINEMATICS_H

#include <stdint.h>

// X 型麦轮运动学纯函数层（无硬件依赖，PC 可单测）。
// 推导与符号表：docs/V2_MECANUM_S3.md §2/§3。坐标系：x 前、y 左、wz 逆时针为正。
// 轮序固定：FL, FR, RL, RR。

namespace mecanum {

enum WheelIndex {
  WHEEL_FL = 0,
  WHEEL_FR = 1,
  WHEEL_RL = 2,
  WHEEL_RR = 3,
  WHEEL_COUNT = 4,
};

struct Twist {
  float vx;  // m/s, + = 前进
  float vy;  // m/s, + = 左移
  float wz;  // rad/s, + = 逆时针（俯视 CCW）
};

struct WheelSpeeds {
  // 轮缘线速度 m/s。+ = 该轮正转（单独驱动时使车体 +x 前进）。
  float v[WHEEL_COUNT];
};

// 逆运动学：twist -> 四轮线速度。文档 §2.3 式 (1)-(4)，K = halfWheelbase + halfTrack。
// 返回 false 表示几何参数非法（<=0）。
bool inverse(const Twist& t, float halfWheelbaseM, float halfTrackM, WheelSpeeds& out);

// 正运动学：四轮 -> twist（最小二乘；四轮速度一致时为精确解）。文档 §2.4。
bool forward(const WheelSpeeds& w, float halfWheelbaseM, float halfTrackM, Twist& out);

// 统一反解饱和：四轮同乘一个系数使 max|v| <= vMax。
// 保方向（vx:vy:wz 比例不变），弃幅值。返回实际施加的缩放（<=1）。
float desaturateUniform(WheelSpeeds& w, float vMaxMps);

// twist 整体限幅：任一分量超限时全体等比缩放（保方向）。返回缩放（<=1）。
float clampTwistUniform(Twist& t, float vxMaxMps, float vyMaxMps, float wzMaxRads);

// V1 DIFF 兼容映射：PWM 对 -> twist。文档 §6。
// pwmL/pwmR ∈ [0,255]（V1 语义：两轮均向前，无反转）；超界钳位。
// vx = (uL+uR)/2, wz = (uR-uL)/trackM（差速轨距，非麦轮 K），vy ≡ 0。
bool diffToTwist(int pwmL, int pwmR, float diffMaxMps, float trackM, Twist& out);

// V1 MOVE 兼容映射：F/B -> vx；L/R -> 原地转 wz（V1 搜索转向语义）。文档 §6。
bool moveToTwist(char dirUpper, int pwm, float moveMaxMps, float pivotWzRadS, Twist& out);

// 编码器计数增量 -> 轮缘线速度（m/s）。cpr = 轮轴每转计数（含减速比与正交倍频）。
float countsToMps(int32_t deltaCounts, float wheelCpr, float wheelRadiusM, float dtS);

}  // namespace mecanum

#endif  // MECANUM_KINEMATICS_H
