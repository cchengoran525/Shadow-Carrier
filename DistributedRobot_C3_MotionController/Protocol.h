#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>

namespace RobotProtocol {

enum class CommandType {
  Move,
  Stop,
  Ping,
  Invalid
};

enum class MoveDirection {
  Forward,
  Back,
  Left,
  Right,
  Unknown
};

struct Command {
  CommandType type = CommandType::Invalid;
  MoveDirection direction = MoveDirection::Unknown;
  int speed = 0;
};

}  // namespace RobotProtocol

#endif
