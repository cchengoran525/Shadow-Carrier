#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>

namespace RobotProtocol {

// Protocol commands are ASCII text, one command per line.
constexpr char COMMAND_FORWARD[] = "MOVE F 180";
constexpr char COMMAND_BACK[] = "MOVE B 180";
constexpr char COMMAND_LEFT[] = "MOVE L 150";
constexpr char COMMAND_RIGHT[] = "MOVE R 150";
constexpr char COMMAND_STOP[] = "STOP";
constexpr char COMMAND_PING[] = "PING";

}  // namespace RobotProtocol

#endif
