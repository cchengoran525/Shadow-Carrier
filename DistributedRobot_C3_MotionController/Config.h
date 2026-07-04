#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Serial monitor baud rate for debugging.
constexpr uint32_t DEBUG_BAUD_RATE = 115200;

// UART link from ESP32-S3 network gateway to ESP32-C3 motion controller.
constexpr uint32_t MOTION_UART_BAUD_RATE = 115200;
constexpr int MOTION_UART_RX_PIN = 10;
constexpr int MOTION_UART_TX_PIN = 11;

// Maximum command length. Protocol commands are intentionally short ASCII lines.
constexpr size_t MAX_COMMAND_LENGTH = 48;

#endif
