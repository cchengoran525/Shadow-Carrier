#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>

class UltrasonicSensor {
 public:
  void begin();
  void update();
  bool isObstacleDetected() const;
  int lastDistanceCm() const;

 private:
  int measureDistanceCm();

  uint32_t lastSampleMs_ = 0;
  int lastDistanceCm_ = -1;
  uint8_t clearSampleCount_ = 0;
  bool obstacleDetected_ = false;
};

void initUltrasonicSensor(UltrasonicSensor &sensor);

#endif
