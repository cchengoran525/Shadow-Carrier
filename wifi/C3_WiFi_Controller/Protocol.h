#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>

namespace RobotProtocol {

enum class CommandType {
  Move,
  Stop,
  Ping,
  Diff,    // 差速: DIFF L100 R70
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
  int leftSpeed = 0;   // DIFF 命令的左右轮速度
  int rightSpeed = 0;
};

}  // namespace RobotProtocol

#endif
