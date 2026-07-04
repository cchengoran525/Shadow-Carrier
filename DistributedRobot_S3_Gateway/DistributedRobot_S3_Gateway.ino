#include <Arduino.h>

#include "Config.h"
#include "UartGateway.h"
#include "WebServerHandlers.h"
#include "WifiGateway.h"

UartGateway motionUart;
WebServerHandlers webHandlers(motionUart);

void setup() {
  Serial.begin(DEBUG_BAUD_RATE);
  delay(200);

  Serial.println();
  Serial.println("Distributed Robot Platform - ESP32-S3 Network Gateway v0.2");

  initUART(motionUart);
  initWiFi();
  initWebServer(webHandlers);
}

void loop() {
  handleWebClient();
}
