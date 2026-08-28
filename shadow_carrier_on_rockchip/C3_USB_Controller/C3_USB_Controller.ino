/**
 * ShadowCarrier C3 USB Motion Controller (FINAL) + Gimbal
 * C3 通过 USB 线直连 KickPi, Serial(CDC ACM) 接收 ASCII 命令驱动 TB6612。
 * 无需 CH340/WiFi/板载UART。USB 即供电又通信。
 *
 * 云台 (v0.5): 两路 50Hz 硬件 PWM (LEDC) 直驱舵机
 *   PAN <deg>  水平 (180° 舵机, GPIO0)
 *   TLT <deg>  俯仰 (180° 舵机, GPIO1)
 *   角度超范围自动钳制; 上电回中位。
 *
 * ⚠️ 舵机电源必须独立 5V 供电 (峰值电流 >1A), GND 与 C3 共地。
 * 依赖: CommandParser.h/cpp, Config.h, MotorDriver.h/cpp, UltrasonicSensor.h/cpp
 */
#include <Arduino.h>
#include "CommandParser.h"
#include "Config.h"
#include "MotorDriver.h"
#include "UltrasonicSensor.h"

// ===================== 云台配置 =====================
constexpr int GIMBAL_PAN_PIN = 0;   // Pan 信号 (橙线) -> GPIO0
constexpr int GIMBAL_TILT_PIN = 1;  // Tilt 信号 (橙线) -> GPIO1

constexpr float PAN_RANGE_DEG = 180.0f;  // 水平舵机行程 (实测180°舵机)
constexpr float TILT_RANGE_DEG = 180.0f; // 俯仰舵机行程
constexpr float PAN_INIT_DEG = 90.0f;    // 上电中位
constexpr float TILT_INIT_DEG = 90.0f;

constexpr bool PAN_INVERT = true;   // Pan 方向标定: 拖右转左就改这里
constexpr bool TILT_INVERT = false;

constexpr uint32_t SERVO_FREQ_HZ = 50;      // 标准舵机 50Hz, 周期 20ms
constexpr uint8_t SERVO_RESOLUTION_BITS = 14;
constexpr uint32_t SERVO_PERIOD_US = 20000;
constexpr uint32_t SERVO_PULSE_MIN_US = 500;   // 0.5ms
constexpr uint32_t SERVO_PULSE_MAX_US = 2500;  // 2.5ms
// 若某轴方向反了/行程不对, 单独调这两个值即可
// ===================================================

CommandParser parser;
MotorDriver motor;
UltrasonicSensor ultrasonic;
String cmdBuf;

namespace {

struct ServoChannel {
  int pin;
  float rangeDeg;
  float angle;
  bool invert;
};

ServoChannel panServo = {GIMBAL_PAN_PIN, PAN_RANGE_DEG, PAN_INIT_DEG, PAN_INVERT};
ServoChannel tiltServo = {GIMBAL_TILT_PIN, TILT_RANGE_DEG, TILT_INIT_DEG, TILT_INVERT};

uint32_t angleToDuty(const ServoChannel &s) {
  float eff = s.angle;
  if (s.invert) {
    eff = s.rangeDeg - eff;  // 镜像到中位另一侧, 方向反转
  }
  if (eff < 0) eff = 0;
  if (eff > s.rangeDeg) eff = s.rangeDeg;
  const float pulseUs =
      SERVO_PULSE_MIN_US +
      (eff / s.rangeDeg) * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US);
  return static_cast<uint32_t>(pulseUs * ((1 << SERVO_RESOLUTION_BITS) - 1) /
                               SERVO_PERIOD_US);
}

void servoInit(ServoChannel &s) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(s.pin, SERVO_FREQ_HZ, SERVO_RESOLUTION_BITS);
  ledcWrite(s.pin, angleToDuty(s));
#else
  constexpr int PAN_CH = 0;
  constexpr int TILT_CH = 1;
  const int ch = (s.pin == GIMBAL_PAN_PIN) ? PAN_CH : TILT_CH;
  ledcSetup(ch, SERVO_FREQ_HZ, SERVO_RESOLUTION_BITS);
  ledcAttachPin(s.pin, ch);
  ledcWrite(ch, angleToDuty(s));
#endif
}

void servoSet(ServoChannel &s, float deg) {
  if (deg < 0) deg = 0;
  if (deg > s.rangeDeg) deg = s.rangeDeg;
  s.angle = deg;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(s.pin, angleToDuty(s));
#else
  const int ch = (s.pin == GIMBAL_PAN_PIN) ? 0 : 1;
  ledcWrite(ch, angleToDuty(s));
#endif
}

// 返回 true 表示该行是云台命令(已处理), false 则交给原有运动解析
bool handleGimbalCommand(const String &line) {
  int spaceIndex = line.indexOf(' ');
  if (spaceIndex <= 0) {
    return false;
  }
  String action = line.substring(0, spaceIndex);
  String valueToken = line.substring(spaceIndex + 1);
  valueToken.trim();
  if (!isDigit(valueToken[0]) && valueToken[0] != '-') {
    return false;
  }

  if (action == "PAN") {
    servoSet(panServo, valueToken.toFloat());
    Serial.print("PAN:");
    Serial.println(panServo.angle, 1);
    return true;
  }
  if (action == "TLT") {
    servoSet(tiltServo, valueToken.toFloat());
    Serial.print("TLT:");
    Serial.println(tiltServo.angle, 1);
    return true;
  }
  return false;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("C3_USB_READY");
  initMotorDriver(motor);
  initUltrasonicSensor(ultrasonic);
  servoInit(panServo);
  servoInit(tiltServo);
  cmdBuf.reserve(48);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      cmdBuf.trim();
      if (cmdBuf.length() > 0) {
        Serial.print("GOT:");
        Serial.println(cmdBuf);
        if (!handleGimbalCommand(cmdBuf)) {
          motor.execute(parser.parse(cmdBuf));
        }
      }
      cmdBuf = "";
    } else if (cmdBuf.length() < 48) {
      cmdBuf += c;
    }
  }
  ultrasonic.update();
  motor.setObstacleDetected(ultrasonic.isObstacleDetected());
  motor.update();

  // 超声波遥测: 状态变化即报 + 500ms心跳 (供RK避障决策)
  static uint32_t lastObsPub = 0;
  static bool lastObsPubState = false;
  bool obsNow = ultrasonic.isObstacleDetected();
  if (obsNow != lastObsPubState || millis() - lastObsPub > 500) {
    Serial.print("OBS ");
    Serial.print(obsNow ? 1 : 0);
    Serial.print(" ");
    Serial.println(ultrasonic.lastDistanceCm());
    lastObsPubState = obsNow;
    lastObsPub = millis();
  }
}
