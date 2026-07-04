#include <Arduino.h>

#include "CommandParser.h"
#include "Config.h"
#include "MotionUart.h"

CommandParser commandParser;

void setup() {
  Serial.begin(DEBUG_BAUD_RATE);
  delay(200);

  Serial.println();
  Serial.println("Distributed Robot Platform - ESP32-C3 Motion Controller v0.2");

  initMotionUART();
}

void loop() {
  processIncomingCommands(commandParser);
}
