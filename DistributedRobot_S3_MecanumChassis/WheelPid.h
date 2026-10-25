#ifndef WHEEL_PID_H
#define WHEEL_PID_H

// 单轮速度环 PID（纯类，无硬件依赖，PC 可单测）。
// 位置式 P + I + D(对测量微分)，积分钳位 + 输出饱和时停积分（anti-windup）。
// 输入目标/测量均为轮缘线速度 m/s（正 = 驱车前进），输出 duty ∈ [outMin, outMax]。

class WheelPid {
 public:
  void configure(float kp, float ki, float kd, float outMin, float outMax, float iTermMax);

  void setTarget(float mps);
  float target() const { return targetMps_; }

  // 推进一个控制周期；dt 单位秒。返回 duty（已钳位）。
  float step(float measMps, float dtS);

  // 清目标与内部状态（STOP / 超时 / 故障时调用）。
  void reset();

 private:
  float kp_ = 0.0f, ki_ = 0.0f, kd_ = 0.0f;
  float outMin_ = -1.0f, outMax_ = 1.0f, iTermMax_ = 0.4f;
  float targetMps_ = 0.0f;
  float iTerm_ = 0.0f;
  float lastMeas_ = 0.0f;
  bool hasLastMeas_ = false;
};

#endif  // WHEEL_PID_H
