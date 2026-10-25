#include "WheelPid.h"

#include <cmath>

void WheelPid::configure(float kp, float ki, float kd, float outMin, float outMax, float iTermMax) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  outMin_ = outMin;
  outMax_ = outMax;
  iTermMax_ = fabsf(iTermMax);
}

void WheelPid::setTarget(float mps) {
  targetMps_ = mps;
}

void WheelPid::reset() {
  targetMps_ = 0.0f;
  iTerm_ = 0.0f;
  lastMeas_ = 0.0f;
  hasLastMeas_ = false;
}

float WheelPid::step(float measMps, float dtS) {
  if (dtS <= 0.0f) {
    return 0.0f;
  }
  const float error = targetMps_ - measMps;

  // 先算不含新积分的输出，用于判断饱和方向（条件积分）。
  float dTerm = 0.0f;
  if (hasLastMeas_) {
    dTerm = -kd_ * (measMps - lastMeas_) / dtS;
  }
  const float unclamped = kp_ * error + iTerm_ + dTerm;

  // 积分项：已饱和且误差继续推向饱和方向时不积分（anti-windup）。
  const bool railedHigh = unclamped >= outMax_ && error > 0.0f;
  const bool railedLow = unclamped <= outMin_ && error < 0.0f;
  if (!railedHigh && !railedLow) {
    iTerm_ += ki_ * error * dtS;
    if (iTerm_ > iTermMax_) iTerm_ = iTermMax_;
    if (iTerm_ < -iTermMax_) iTerm_ = -iTermMax_;
  }

  lastMeas_ = measMps;
  hasLastMeas_ = true;

  float out = kp_ * error + iTerm_ + dTerm;
  if (out > outMax_) out = outMax_;
  if (out < outMin_) out = outMin_;
  return out;
}
