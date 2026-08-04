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

// Open-loop motor safety and tuning.
constexpr uint32_t MOTOR_COMMAND_TIMEOUT_MS = 450;
constexpr uint32_t MOTOR_RAMP_INTERVAL_MS = 20;
constexpr int MOTOR_RAMP_STEP = 8;

// Channel A is treated as the left motor, channel B as the right motor.
// Use small trim values such as -5, 0, or 5 to correct open-loop drift.
constexpr int MOTOR_LEFT_TRIM_PERCENT = 0;
constexpr int MOTOR_RIGHT_TRIM_PERCENT = 0;

// Front ultrasonic obstacle sensor.
constexpr int ULTRASONIC_TRIG_PIN = 2;
constexpr int ULTRASONIC_ECHO_PIN = 3;
constexpr uint32_t ULTRASONIC_SAMPLE_INTERVAL_MS = 80;
constexpr uint32_t ULTRASONIC_ECHO_TIMEOUT_US = 12000;
constexpr int OBSTACLE_STOP_DISTANCE_CM = 20;
constexpr int OBSTACLE_CLEAR_DISTANCE_CM = 25;
constexpr uint8_t OBSTACLE_CLEAR_CONFIRM_SAMPLES = 3;

// Maximum command length. Protocol commands are intentionally short ASCII lines.
constexpr size_t MAX_COMMAND_LENGTH = 48;

#endif
