#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Serial monitor baud rate for debugging.
constexpr uint32_t DEBUG_BAUD_RATE = 115200;

// UART link from ESP32-S3 network gateway to ESP32-C3 motion controller.
constexpr uint32_t MOTION_UART_BAUD_RATE = 115200;
constexpr int MOTION_UART_RX_PIN = 10;
constexpr int MOTION_UART_TX_PIN = 11;

// TB6612 motor driver pins.
constexpr int MOTOR_STBY_PIN = 20;
constexpr int MOTOR_AIN1_PIN = 4;
constexpr int MOTOR_AIN2_PIN = 5;
constexpr int MOTOR_PWMA_PIN = 6;
constexpr int MOTOR_BIN1_PIN = 7;
constexpr int MOTOR_BIN2_PIN = 8;
constexpr int MOTOR_PWMB_PIN = 9;

// Maximum command length. Protocol commands are intentionally short ASCII lines.
constexpr size_t MAX_COMMAND_LENGTH = 48;

#endif
