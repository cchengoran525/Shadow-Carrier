/**
 * ShadowCarrier C3 WiFi Motion Controller v0.3
 * ESP32-C3 连 KickPi 热点, TCP:8888 接收 ASCII 命令, 驱动 TB6612。
 * 保留: 电机斜坡/超声波避障/450ms超时/STBY管理。
 * 依赖: 原 C3 固件的 CommandParser.h/cpp, Config.h, MotorDriver.h/cpp, UltrasonicSensor.h/cpp
 */
#include <WiFi.h>
#include "CommandParser.h"
#include "Config.h"
#include "MotorDriver.h"
#include "UltrasonicSensor.h"

const char* WIFI_SSID = "ShadowCarrier-RK";
const char* WIFI_PASS = "shadow123456";
const uint16_t TCP_PORT = 8888;

WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;

CommandParser parser;
MotorDriver motor;
UltrasonicSensor ultrasonic;

String cmdBuffer;
static const size_t MAX_CMD = 48;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== ShadowCarrier C3 WiFi v0.3 ===");

  initMotorDriver(motor);
  initUltrasonicSensor(ultrasonic);
  cmdBuffer.reserve(MAX_CMD);

  WiFi.mode(WIFI_STA);
  WiFi.setHostname("shadow-c3");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("WiFi: %s ...\n", WIFI_SSID);
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 60) {
    delay(500); Serial.print("."); retry++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWiFi OK  IP=%s\n", WiFi.localIP().toString().c_str());
    tcpServer.begin();
    Serial.printf("TCP server :%d\n", TCP_PORT);
  } else {
    Serial.println("\nWiFi FAIL");
  }
}

void processCommand(const String& line) {
  if (line.length() == 0) return;
  Serial.printf("TCP<-: %s\n", line.c_str());
  RobotProtocol::Command cmd = parser.parse(line);
  parser.printCommand(cmd);
  motor.execute(cmd);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost, reconnecting...");
    WiFi.reconnect();
    delay(3000);
    return;
  }

  if (!tcpClient || !tcpClient.connected()) {
    tcpClient = tcpServer.accept();
    if (tcpClient) {
      Serial.printf("TCP client: %s\n", tcpClient.remoteIP().toString().c_str());
      cmdBuffer = "";
      motor.stop();
    }
  }

  if (tcpClient && tcpClient.connected()) {
    while (tcpClient.available() > 0) {
      char c = (char)tcpClient.read();
      if (c == '\r') continue;
      if (c == '\n') {
        cmdBuffer.trim();
        processCommand(cmdBuffer);
        cmdBuffer = "";
      } else if (cmdBuffer.length() < MAX_CMD) {
        cmdBuffer += c;
      } else {
        cmdBuffer = "";
      }
    }
  }

  ultrasonic.update();
  motor.setObstacleDetected(ultrasonic.isObstacleDetected());
  motor.update();
}
