#include "UartGateway.h"

#include "Config.h"

void UartGateway::begin(uint32_t baudRate, int rxPin, int txPin) {
  serialPort_->begin(baudRate, SERIAL_8N1, rxPin, txPin);

  Serial.print("Motion UART ready. RX=");
  Serial.print(rxPin);
  Serial.print(" TX=");
  Serial.print(txPin);
  Serial.print(" Baud=");
  Serial.println(baudRate);
}

void UartGateway::sendCommand(const char *command) {
  if (command == nullptr) {
    return;
  }

  // Every protocol command is terminated by a newline.
  serialPort_->println(command);

  Serial.print("UART -> C3: ");
  Serial.println(command);
}

void initUART(UartGateway &gateway) {
  gateway.begin(MOTION_UART_BAUD_RATE, MOTION_UART_RX_PIN, MOTION_UART_TX_PIN);
}
