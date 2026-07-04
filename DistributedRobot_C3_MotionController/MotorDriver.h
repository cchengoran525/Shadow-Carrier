#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

#include "Protocol.h"

class MotorDriver {
 public:
  void begin();
  void execute(const RobotProtocol::Command &command);

  void forward(int speed);
  void backward(int speed);
  void left(int speed);
  void right(int speed);
  void stop();

 private:
  void setStandby(bool enabled);
  void setMotorPins(bool aIn1, bool aIn2, bool bIn1, bool bIn2, int speed);
  int clampSpeed(int speed) const;
};

void initMotorDriver(MotorDriver &motorDriver);

#endif
