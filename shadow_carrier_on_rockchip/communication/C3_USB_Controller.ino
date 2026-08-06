/**
 * ShadowCarrier C3 USB Motion Controller (FINAL)
 * C3 USB直连KickPi, Serial(CDC ACM)收ASCII命令驱TB6612。
 * 无需CH340/WiFi/板载UART。USB即供电又通信。
 * 依赖: CommandParser.h/cpp, Config.h, MotorDriver.h/cpp, UltrasonicSensor.h/cpp
 */
#include <WiFi.h>
#include "CommandParser.h"
#include "Config.h"
#include "MotorDriver.h"
#include "UltrasonicSensor.h"

CommandParser parser;
MotorDriver motor;
UltrasonicSensor ultrasonic;
String cmdBuf;

void setup() {
  Serial.begin(115200); delay(300);
  Serial.println("C3_USB_READY");
  initMotorDriver(motor);
  initUltrasonicSensor(ultrasonic);
  cmdBuf.reserve(48);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      cmdBuf.trim();
      if (cmdBuf.length()) { Serial.print("GOT:"); Serial.println(cmdBuf); motor.execute(parser.parse(cmdBuf)); }
      cmdBuf = "";
    } else if (cmdBuf.length() < 48) cmdBuf += c;
  }
  ultrasonic.update();
  motor.setObstacleDetected(ultrasonic.isObstacleDetected());
  motor.update();
}
