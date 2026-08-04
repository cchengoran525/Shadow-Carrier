#include "MotorDriver.h"

#include "Config.h"

void MotorDriver::begin() {
  // Keep the TB6612 disabled before any direction or PWM pin can move.
  digitalWrite(MOTOR_STBY_PIN, LOW);
  pinMode(MOTOR_STBY_PIN, OUTPUT);

  digitalWrite(MOTOR_AIN1_PIN, LOW);
  digitalWrite(MOTOR_AIN2_PIN, LOW);
  digitalWrite(MOTOR_PWMA_PIN, LOW);
  digitalWrite(MOTOR_BIN1_PIN, LOW);
  digitalWrite(MOTOR_BIN2_PIN, LOW);
  digitalWrite(MOTOR_PWMB_PIN, LOW);

  pinMode(MOTOR_AIN1_PIN, OUTPUT);
  pinMode(MOTOR_AIN2_PIN, OUTPUT);
  pinMode(MOTOR_PWMA_PIN, OUTPUT);
  pinMode(MOTOR_BIN1_PIN, OUTPUT);
  pinMode(MOTOR_BIN2_PIN, OUTPUT);
  pinMode(MOTOR_PWMB_PIN, OUTPUT);

  stop();
  setStandby(false);
  delay(300);

  Serial.println("TB6612 motor driver ready.");
}

void MotorDriver::update() {
  uint32_t now = millis();

  if (targetMode_ != MotionMode::Stopped && now - lastCommandMs_ > MOTOR_COMMAND_TIMEOUT_MS) {
    Serial.println("Motor command timeout, stopping.");
    stop();
  }

  if (obstacleDetected_ && targetMode_ == MotionMode::Forward) {
    Serial.println("Obstacle detected, forward motion stopped.");
    stop();
  }

  if (now - lastRampMs_ < MOTOR_RAMP_INTERVAL_MS) {
    return;
  }
  lastRampMs_ = now;

  if (targetMode_ != activeMode_ && currentSpeed_ > 0) {
    currentSpeed_ -= MOTOR_RAMP_STEP;
    if (currentSpeed_ < 0) {
      currentSpeed_ = 0;
    }
    applyOutput();
    return;
  }

  if (targetMode_ != activeMode_) {
    activeMode_ = targetMode_;
  }

  if (currentSpeed_ < targetSpeed_) {
    currentSpeed_ += MOTOR_RAMP_STEP;
    if (currentSpeed_ > targetSpeed_) {
      currentSpeed_ = targetSpeed_;
    }
  } else if (currentSpeed_ > targetSpeed_) {
    currentSpeed_ -= MOTOR_RAMP_STEP;
    if (currentSpeed_ < targetSpeed_) {
      currentSpeed_ = targetSpeed_;
    }
  }

  applyOutput();
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
  if (obstacleDetected_) {
    Serial.println("Forward command ignored because obstacle is too close.");
    stop();
    return;
  }
  requestMotion(MotionMode::Forward, speed);
}

void MotorDriver::backward(int speed) {
  requestMotion(MotionMode::Backward, speed);
}

void MotorDriver::left(int speed) {
  requestMotion(MotionMode::Left, speed);
}

void MotorDriver::right(int speed) {
  requestMotion(MotionMode::Right, speed);
}

void MotorDriver::stop() {
  targetMode_ = MotionMode::Stopped;
  activeMode_ = MotionMode::Stopped;
  targetSpeed_ = 0;
  currentSpeed_ = 0;
  setStandby(false);
  digitalWrite(MOTOR_AIN1_PIN, LOW);
  digitalWrite(MOTOR_AIN2_PIN, LOW);
  digitalWrite(MOTOR_BIN1_PIN, LOW);
  digitalWrite(MOTOR_BIN2_PIN, LOW);
  analogWrite(MOTOR_PWMA_PIN, 0);
  analogWrite(MOTOR_PWMB_PIN, 0);
}

void MotorDriver::setObstacleDetected(bool detected) {
  if (detected && !obstacleDetected_ &&
      (targetMode_ == MotionMode::Forward || activeMode_ == MotionMode::Forward)) {
    Serial.println("Obstacle latch engaged, stopping forward motion immediately.");
    stop();
  }
  obstacleDetected_ = detected;
}

void MotorDriver::requestMotion(MotionMode mode, int speed) {
  targetMode_ = mode;
  targetSpeed_ = clampSpeed(speed);
  lastCommandMs_ = millis();

  if (activeMode_ == MotionMode::Stopped) {
    activeMode_ = mode;
  }
}

void MotorDriver::applyOutput() {
  int leftSpeed = applyTrim(currentSpeed_, MOTOR_LEFT_TRIM_PERCENT);
  int rightSpeed = applyTrim(currentSpeed_, MOTOR_RIGHT_TRIM_PERCENT);

  switch (activeMode_) {
    case MotionMode::Forward:
      setMotorPins(true, false, true, false, leftSpeed, rightSpeed);
      break;

    case MotionMode::Backward:
      setMotorPins(false, true, false, true, leftSpeed, rightSpeed);
      break;

    case MotionMode::Left:
      setMotorPins(false, true, true, false, leftSpeed, rightSpeed);
      break;

    case MotionMode::Right:
      setMotorPins(true, false, false, true, leftSpeed, rightSpeed);
      break;

    case MotionMode::Stopped:
    default:
      stop();
      break;
  }
}

void MotorDriver::setStandby(bool enabled) {
  digitalWrite(MOTOR_STBY_PIN, enabled ? HIGH : LOW);
}

void MotorDriver::setMotorPins(bool aIn1, bool aIn2, bool bIn1, bool bIn2, int leftSpeed, int rightSpeed) {
  digitalWrite(MOTOR_AIN1_PIN, aIn1 ? HIGH : LOW);
  digitalWrite(MOTOR_AIN2_PIN, aIn2 ? HIGH : LOW);
  digitalWrite(MOTOR_BIN1_PIN, bIn1 ? HIGH : LOW);
  digitalWrite(MOTOR_BIN2_PIN, bIn2 ? HIGH : LOW);
  analogWrite(MOTOR_PWMA_PIN, clampSpeed(leftSpeed));
  analogWrite(MOTOR_PWMB_PIN, clampSpeed(rightSpeed));
  setStandby(leftSpeed > 0 || rightSpeed > 0);
}

int MotorDriver::applyTrim(int speed, int trimPercent) const {
  return clampSpeed(speed + speed * trimPercent / 100);
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
