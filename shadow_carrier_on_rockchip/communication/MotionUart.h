#ifndef MOTION_UART_H
#define MOTION_UART_H

#include "CommandParser.h"
#include "MotorDriver.h"

void initMotionUART();
void processIncomingCommands(CommandParser &parser, MotorDriver &motorDriver);

#endif
