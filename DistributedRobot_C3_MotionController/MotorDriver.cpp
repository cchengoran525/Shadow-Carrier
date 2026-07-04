#include "MotorDriver.h"

#include "Config.h"

void MotorDriver::begin() {
  pinMode(MOTOR_STBY_PIN, OUTPUT);
  pinMode(MOTOR_AIN1_PIN, OUTPUT);
  pinMode(MOTOR_AIN2_PIN, OUTPUT);
  pinMode(MOTOR_PWMA_PIN, OUTPUT);
  pinMode(MOTOR_BIN1_PIN, OUTPUT);
  pinMode(MOTOR_BIN2_PIN, OUTPUT);
  pinMode(MOTOR_PWMB_PIN, OUTPUT);

  stop();
  setStandby(false);
  delay(500);
  setStandby(true);

  Serial.println("TB6612 motor driver ready.");
}

void MotorDriver::execute(const RobotProtocol::Command &command) {
  switch (command.type) {
    case RobotProtocol::CommandType::Move:
      switch (command.direction) {
        case RobotProtocol::MoveDirection::Forward:
          forward(command.speed);
          break;
        case RobotProtocol::MoveDirection::Back:
          backward(command.speed);
          break;
        case RobotProtocol::MoveDirection::Left:
          left(command.speed);
          break;
        case RobotProtocol::MoveDirection::Right:
          right(command.speed);
          break;
        case RobotProtocol::MoveDirection::Unknown:
        default:
          stop();
          break;
      }
      break;

    case RobotProtocol::CommandType::Stop:
      stop();
      break;

    case RobotProtocol::CommandType::Ping:
      Serial.println("PONG");
      break;

    case RobotProtocol::CommandType::Invalid:
    default:
      stop();
      break;
  }
}

void MotorDriver::forward(int speed) {
  setMotorPins(true, false, true, false, speed);
}

void MotorDriver::backward(int speed) {
  setMotorPins(false, true, false, true, speed);
}

void MotorDriver::left(int speed) {
  setMotorPins(false, true, true, false, speed);
}

void MotorDriver::right(int speed) {
  setMotorPins(true, false, false, true, speed);
}

void MotorDriver::stop() {
  digitalWrite(MOTOR_AIN1_PIN, LOW);
  digitalWrite(MOTOR_AIN2_PIN, LOW);
  digitalWrite(MOTOR_BIN1_PIN, LOW);
  digitalWrite(MOTOR_BIN2_PIN, LOW);
  analogWrite(MOTOR_PWMA_PIN, 0);
  analogWrite(MOTOR_PWMB_PIN, 0);
}

void MotorDriver::setStandby(bool enabled) {
  digitalWrite(MOTOR_STBY_PIN, enabled ? HIGH : LOW);
}

void MotorDriver::setMotorPins(bool aIn1, bool aIn2, bool bIn1, bool bIn2, int speed) {
  int pwm = clampSpeed(speed);

  digitalWrite(MOTOR_AIN1_PIN, aIn1 ? HIGH : LOW);
  digitalWrite(MOTOR_AIN2_PIN, aIn2 ? HIGH : LOW);
  digitalWrite(MOTOR_BIN1_PIN, bIn1 ? HIGH : LOW);
  digitalWrite(MOTOR_BIN2_PIN, bIn2 ? HIGH : LOW);
  analogWrite(MOTOR_PWMA_PIN, pwm);
  analogWrite(MOTOR_PWMB_PIN, pwm);
}

int MotorDriver::clampSpeed(int speed) const {
  if (speed < 0) {
    return 0;
  }
  if (speed > 255) {
    return 255;
  }
  return speed;
}

void initMotorDriver(MotorDriver &motorDriver) {
  motorDriver.begin();
}
