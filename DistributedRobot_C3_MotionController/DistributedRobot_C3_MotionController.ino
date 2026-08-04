#include <Arduino.h>

#include "CommandParser.h"
#include "Config.h"
#include "MotorDriver.h"
#include "MotionUart.h"
#include "UltrasonicSensor.h"

CommandParser commandParser;
MotorDriver motorDriver;
UltrasonicSensor ultrasonicSensor;

void setup() {
  Serial.begin(DEBUG_BAUD_RATE);
  delay(200);

  Serial.println();
  Serial.println("Distributed Robot Platform - ESP32-C3 Motion Controller v0.2");

  initMotorDriver(motorDriver);
  initUltrasonicSensor(ultrasonicSensor);
  initMotionUART();
}

void loop() {
  processIncomingCommands(commandParser, motorDriver);
  ultrasonicSensor.update();
  motorDriver.setObstacleDetected(ultrasonicSensor.isObstacleDetected());
  motorDriver.update();
}
