#include "MecanumKinematics.h"

#include <cmath>

namespace mecanum {

namespace {
inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}
constexpr float kTwoPi = 6.28318530717958647692f;
}  // namespace

bool inverse(const Twist& t, float halfWheelbaseM, float halfTrackM, WheelSpeeds& out) {
  if (halfWheelbaseM <= 0.0f || halfTrackM <= 0.0f) {
    return false;
  }
  const float k = halfWheelbaseM + halfTrackM;  // K = L + W
  // 文档 §2.3 式 (1)-(4)。X 型：FL/RR 辊线 "\"，FR/RL 辊线 "/"。
  out.v[WHEEL_FL] = t.vx - t.vy - k * t.wz;
  out.v[WHEEL_FR] = t.vx + t.vy + k * t.wz;
  out.v[WHEEL_RL] = t.vx + t.vy - k * t.wz;
  out.v[WHEEL_RR] = t.vx - t.vy + k * t.wz;
  return true;
}

bool forward(const WheelSpeeds& w, float halfWheelbaseM, float halfTrackM, Twist& out) {
  if (halfWheelbaseM <= 0.0f || halfTrackM <= 0.0f) {
    return false;
  }
  const float k = halfWheelbaseM + halfTrackM;
  const float uFl = w.v[WHEEL_FL];
  const float uFr = w.v[WHEEL_FR];
  const float uRl = w.v[WHEEL_RL];
  const float uRr = w.v[WHEEL_RR];
  out.vx = (uFl + uFr + uRl + uRr) * 0.25f;
  out.vy = (-uFl + uFr + uRl - uRr) * 0.25f;
  out.wz = (-uFl + uFr - uRl + uRr) / (4.0f * k);
  return true;
}

float desaturateUniform(WheelSpeeds& w, float vMaxMps) {
  if (vMaxMps <= 0.0f) {
    for (int i = 0; i < WHEEL_COUNT; ++i) w.v[i] = 0.0f;
    return 0.0f;
  }
  float peak = 0.0f;
  for (int i = 0; i < WHEEL_COUNT; ++i) {
    peak = fmaxf(peak, fabsf(w.v[i]));
  }
  if (peak <= vMaxMps) {
    return 1.0f;
  }
  const float scale = vMaxMps / peak;
  for (int i = 0; i < WHEEL_COUNT; ++i) {
    w.v[i] *= scale;
  }
  return scale;
}

float clampTwistUniform(Twist& t, float vxMaxMps, float vyMaxMps, float wzMaxRads) {
  float scale = 1.0f;
  if (vxMaxMps > 0.0f && fabsf(t.vx) > vxMaxMps) scale = fminf(scale, vxMaxMps / fabsf(t.vx));
  if (vyMaxMps > 0.0f && fabsf(t.vy) > vyMaxMps) scale = fminf(scale, vyMaxMps / fabsf(t.vy));
  if (wzMaxRads > 0.0f && fabsf(t.wz) > wzMaxRads) scale = fminf(scale, wzMaxRads / fabsf(t.wz));
  t.vx *= scale;
  t.vy *= scale;
  t.wz *= scale;
  return scale;
}

bool diffToTwist(int pwmL, int pwmR, float diffMaxMps, float trackM, Twist& out) {
  if (diffMaxMps <= 0.0f || trackM <= 0.0f) {
    return false;
  }
  const float uL = clampf((float)pwmL, 0.0f, 255.0f) / 255.0f * diffMaxMps;
  const float uR = clampf((float)pwmR, 0.0f, 255.0f) / 255.0f * diffMaxMps;
  // wz 用差速轨距（复现 V1 弧线行为），随后由调用方走 MEC 逆解。文档 §6 修订说明。
  out.vx = 0.5f * (uL + uR);
  out.vy = 0.0f;
  out.wz = (uR - uL) / trackM;
  return true;
}

bool moveToTwist(char dirUpper, int pwm, float moveMaxMps, float pivotWzRadS, Twist& out) {
  if (moveMaxMps <= 0.0f) {
    return false;
  }
  const float mag = clampf((float)pwm, 0.0f, 255.0f) / 255.0f;
  out.vx = 0.0f;
  out.vy = 0.0f;
  out.wz = 0.0f;
  switch (dirUpper) {
    case 'F': out.vx = +mag * moveMaxMps; return true;
    case 'B': out.vx = -mag * moveMaxMps; return true;
    case 'L': out.wz = +pivotWzRadS * mag; return true;  // 原地左转
    case 'R': out.wz = -pivotWzRadS * mag; return true;  // 原地右转
    default: return false;
  }
}

float countsToMps(int32_t deltaCounts, float wheelCpr, float wheelRadiusM, float dtS) {
  if (wheelCpr <= 0.0f || wheelRadiusM <= 0.0f || dtS <= 0.0f) {
    return 0.0f;
  }
  const float revs = (float)deltaCounts / wheelCpr;
  return revs * (kTwoPi * wheelRadiusM) / dtS;
}

}  // namespace mecanum
