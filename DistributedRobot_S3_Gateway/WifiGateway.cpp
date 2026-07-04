#include "WifiGateway.h"

#include <Arduino.h>
#include <WiFi.h>

#include "Config.h"

void initWiFi() {
  WiFi.mode(WIFI_AP);

  bool apStarted = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);
  if (!apStarted) {
    Serial.println("Failed to start WiFi access point.");
    return;
  }

  Serial.print("WiFi AP started. SSID: ");
  Serial.println(WIFI_AP_SSID);
  Serial.print("Open control page: http://");
  Serial.println(WiFi.softAPIP());
}
