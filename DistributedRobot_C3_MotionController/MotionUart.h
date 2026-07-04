#ifndef MOTION_UART_H
#define MOTION_UART_H

#include "CommandParser.h"

void initMotionUART();
void processIncomingCommands(CommandParser &parser);

#endif
