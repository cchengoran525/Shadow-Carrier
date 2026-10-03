#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

#include "Protocol.h"

class MotorDriver {
 public:
  void begin();
  void update();
  void execute(const RobotProtocol::Command &command);
  void setObstacleDetected(bool detected);

  void forward(int speed);
  void backward(int speed);
  void left(int speed);
  void right(int speed);
  void differential(int leftSpeed, int rightSpeed);
  void stop();

 private:
  enum class MotionMode {
    Stopped,
    Forward,
    Backward,
    Left,
    Right,
    Diff
  };

  void requestMotion(MotionMode mode, int speed);
  void applyOutput();
  void setStandby(bool enabled);
  void setMotorPins(bool aIn1, bool aIn2, bool bIn1, bool bIn2, int leftSpeed, int rightSpeed);
  int applyTrim(int speed, int trimPercent) const;
  int clampSpeed(int speed) const;

  MotionMode targetMode_ = MotionMode::Stopped;
  MotionMode activeMode_ = MotionMode::Stopped;
  int targetSpeed_ = 0;
  int currentSpeed_ = 0;
  int targetLeftSpeed_ = 0, targetRightSpeed_ = 0;
  int currentLeftSpeed_ = 0, currentRightSpeed_ = 0;
  uint32_t lastCommandMs_ = 0;
  uint32_t lastRampMs_ = 0;
  bool obstacleDetected_ = false;
};

void initMotorDriver(MotorDriver &motorDriver);

#endif
