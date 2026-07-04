#include "MotionUart.h"

#include <Arduino.h>

#include "Config.h"

namespace {

String receiveBuffer;

void handleLine(const String &line, CommandParser &parser) {
  if (line.length() == 0) {
    return;
  }

  Serial.print("UART <- S3: ");
  Serial.println(line);

  RobotProtocol::Command command = parser.parse(line);
  parser.printCommand(command);
}

}  // namespace

void initMotionUART() {
  Serial1.begin(MOTION_UART_BAUD_RATE, SERIAL_8N1, MOTION_UART_RX_PIN, MOTION_UART_TX_PIN);
  receiveBuffer.reserve(MAX_COMMAND_LENGTH);

  Serial.print("Motion UART ready. RX=");
  Serial.print(MOTION_UART_RX_PIN);
  Serial.print(" TX=");
  Serial.print(MOTION_UART_TX_PIN);
  Serial.print(" Baud=");
  Serial.println(MOTION_UART_BAUD_RATE);
}

void processIncomingCommands(CommandParser &parser) {
  while (Serial1.available() > 0) {
    char incoming = static_cast<char>(Serial1.read());

    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      receiveBuffer.trim();
      handleLine(receiveBuffer, parser);
      receiveBuffer = "";
      return;
    }

    if (receiveBuffer.length() >= MAX_COMMAND_LENGTH) {
      Serial.println("Command too long, buffer cleared.");
      receiveBuffer = "";
      continue;
    }

    receiveBuffer += incoming;
  }
}
