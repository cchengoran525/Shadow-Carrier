#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// SoftAP configuration. Connect your phone/computer to this WiFi.
constexpr char WIFI_AP_SSID[] = "ShadowCarrier-S3";
constexpr char WIFI_AP_PASSWORD[] = "shadow123456";

// Serial monitor baud rate for debugging.
constexpr uint32_t DEBUG_BAUD_RATE = 115200;

// UART link from ESP32-S3 gateway to ESP32-C3 motion controller.
constexpr uint32_t MOTION_UART_BAUD_RATE = 115200;
constexpr int MOTION_UART_TX_PIN = 1;
constexpr int MOTION_UART_RX_PIN = 2;

// HTTP server port.
constexpr uint16_t HTTP_SERVER_PORT = 80;

// Default motion speed used by the web control page.
constexpr int DEFAULT_MOVE_SPEED = 180;
constexpr int DEFAULT_TURN_SPEED = 150;

#endif
