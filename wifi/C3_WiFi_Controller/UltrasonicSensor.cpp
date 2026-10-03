#include "UltrasonicSensor.h"

#include "Config.h"

void UltrasonicSensor::begin() {
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  Serial.print("Ultrasonic sensor ready. TRIG=");
  Serial.print(ULTRASONIC_TRIG_PIN);
  Serial.print(" ECHO=");
  Serial.println(ULTRASONIC_ECHO_PIN);
}

void UltrasonicSensor::update() {
  uint32_t now = millis();
  if (now - lastSampleMs_ < ULTRASONIC_SAMPLE_INTERVAL_MS) {
    return;
  }
  lastSampleMs_ = now;

  lastDistanceCm_ = measureDistanceCm();
  bool previousObstacleState = obstacleDetected_;

  if (lastDistanceCm_ > 0 && lastDistanceCm_ <= OBSTACLE_STOP_DISTANCE_CM) {
    obstacleDetected_ = true;
    clearSampleCount_ = 0;
    lastObstacleLockMs_ = now;
  } else if (lastDistanceCm_ >= OBSTACLE_CLEAR_DISTANCE_CM) {
    if (clearSampleCount_ < OBSTACLE_CLEAR_CONFIRM_SAMPLES) {
      clearSampleCount_++;
    }
    if (clearSampleCount_ >= OBSTACLE_CLEAR_CONFIRM_SAMPLES) {
      obstacleDetected_ = false;
    }
  } else if (lastDistanceCm_ > 0) {
    clearSampleCount_ = 0;
  } else {
    // 传感器超时/无回波: 累计一定次数后强制清除, 防止永久锁死
    if (obstacleDetected_) {
      if (clearSampleCount_ < OBSTACLE_CLEAR_CONFIRM_SAMPLES + 2) {
        clearSampleCount_++;
      }
      if (clearSampleCount_ >= OBSTACLE_CLEAR_CONFIRM_SAMPLES + 2) {
        obstacleDetected_ = false;
        Serial.println("WARN: obstacle force-cleared (sensor timeout)");
      }
    }
  }

  // 安全网: 锁死超过5秒强制清除
  if (obstacleDetected_ && lastObstacleLockMs_ > 0 &&
      now - lastObstacleLockMs_ > MAX_OBSTACLE_LATCH_MS) {
    obstacleDetected_ = false;
    Serial.println("WARN: obstacle force-cleared (max duration)");
  }

  if (obstacleDetected_ != previousObstacleState) {
    Serial.print("Obstacle state changed: ");
    Serial.print(obstacleDetected_ ? "BLOCKED" : "CLEAR");
    Serial.print(" distance_cm=");
    Serial.println(lastDistanceCm_);
  }
}

bool UltrasonicSensor::isObstacleDetected() const {
  return obstacleDetected_;
}

int UltrasonicSensor::lastDistanceCm() const {
  return lastDistanceCm_;
}

int UltrasonicSensor::measureDistanceCm() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  uint32_t durationUs = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ULTRASONIC_ECHO_TIMEOUT_US);
  if (durationUs == 0) {
    return -1;
  }

  return static_cast<int>(durationUs / 58);
}

void initUltrasonicSensor(UltrasonicSensor &sensor) {
  sensor.begin();
}
