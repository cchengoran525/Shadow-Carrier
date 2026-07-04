#ifndef UART_GATEWAY_H
#define UART_GATEWAY_H

#include <Arduino.h>

class UartGateway {
 public:
  void begin(uint32_t baudRate, int rxPin, int txPin);
  void sendCommand(const char *command);

 private:
  HardwareSerial *serialPort_ = &Serial1;
};

void initUART(UartGateway &gateway);

#endif
