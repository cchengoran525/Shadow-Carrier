#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <Arduino.h>

#include "Protocol.h"

class CommandParser {
 public:
  RobotProtocol::Command parse(const String &line) const;
  void printCommand(const RobotProtocol::Command &command) const;

 private:
  RobotProtocol::MoveDirection parseDirection(const String &token) const;
  const char *directionToText(RobotProtocol::MoveDirection direction) const;
};

#endif
