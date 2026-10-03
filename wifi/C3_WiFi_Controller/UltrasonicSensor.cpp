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
  } else if (lastDistanceCm_ >= OBSTACLE_CLEAR_DISTANCE_CM) {
    if (clearSampleCount_ < OBSTACLE_CLEAR_CONFIRM_SAMPLES) {
      clearSampleCount_++;
    }
    if (clearSampleCount_ >= OBSTACLE_CLEAR_CONFIRM_SAMPLES) {
      obstacleDetected_ = false;
    }
  } else if (lastDistanceCm_ > 0) {
    clearSampleCount_ = 0;
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
