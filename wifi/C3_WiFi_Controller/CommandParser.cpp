#include "CommandParser.h"

namespace {

String nextToken(const String &text, int &startIndex) {
  while (startIndex < text.length() && text[startIndex] == ' ') {
    startIndex++;
  }

  int endIndex = text.indexOf(' ', startIndex);
  if (endIndex == -1) {
    String token = text.substring(startIndex);
    startIndex = text.length();
    return token;
  }

  String token = text.substring(startIndex, endIndex);
  startIndex = endIndex + 1;
  return token;
}

bool hasExtraToken(const String &text, int startIndex) {
  while (startIndex < text.length()) {
    if (text[startIndex] != ' ') {
      return true;
    }
    startIndex++;
  }
  return false;
}

}  // namespace

RobotProtocol::Command CommandParser::parse(const String &line) const {
  RobotProtocol::Command command;
  String normalized = line;
  normalized.trim();
  normalized.toUpperCase();

  if (normalized == "STOP") {
    command.type = RobotProtocol::CommandType::Stop;
    return command;
  }

  if (normalized == "PING") {
    command.type = RobotProtocol::CommandType::Ping;
    return command;
  }

  int index = 0;
  String action = nextToken(normalized, index);

  // DIFF L<speed> R<speed>
  if (action == "DIFF") {
    String leftToken = nextToken(normalized, index);
    String rightToken = nextToken(normalized, index);
    if (leftToken.length() >= 2 && leftToken[0] == 'L' &&
        rightToken.length() >= 2 && rightToken[0] == 'R' &&
        !hasExtraToken(normalized, index)) {
      int ls = leftToken.substring(1).toInt();
      int rs = rightToken.substring(1).toInt();
      if (ls >= 0 && ls <= 255 && rs >= 0 && rs <= 255) {
        command.type = RobotProtocol::CommandType::Diff;
        command.leftSpeed = ls;
        command.rightSpeed = rs;
        return command;
      }
    }
    return command;  // parse fail
  }

  if (action != "MOVE") {
    return command;
  }

  String directionToken = nextToken(normalized, index);
  String speedToken = nextToken(normalized, index);
  if (directionToken.length() == 0 || speedToken.length() == 0 || hasExtraToken(normalized, index)) {
    return command;
  }

  RobotProtocol::MoveDirection direction = parseDirection(directionToken);
  int speed = speedToken.toInt();

  if (direction == RobotProtocol::MoveDirection::Unknown || speed < 0 || speed > 255) {
    return command;
  }

  command.type = RobotProtocol::CommandType::Move;
  command.direction = direction;
  command.speed = speed;
  return command;
}

void CommandParser::printCommand(const RobotProtocol::Command &command) const {
  switch (command.type) {
    case RobotProtocol::CommandType::Move:
      Serial.print("Parsed command: MOVE ");
      Serial.print(directionToText(command.direction));
      Serial.print(" speed=");
      Serial.println(command.speed);
      break;

    case RobotProtocol::CommandType::Diff:
      Serial.print("Parsed command: DIFF L");
      Serial.print(command.leftSpeed);
      Serial.print(" R");
      Serial.println(command.rightSpeed);
      break;

    case RobotProtocol::CommandType::Stop:
      Serial.println("Parsed command: STOP");
      break;

    case RobotProtocol::CommandType::Ping:
      Serial.println("Parsed command: PING");
      break;

    case RobotProtocol::CommandType::Invalid:
    default:
      Serial.println("Invalid command");
      break;
  }
}

RobotProtocol::MoveDirection CommandParser::parseDirection(const String &token) const {
  if (token == "F") {
    return RobotProtocol::MoveDirection::Forward;
  }
  if (token == "B") {
    return RobotProtocol::MoveDirection::Back;
  }
  if (token == "L") {
    return RobotProtocol::MoveDirection::Left;
  }
  if (token == "R") {
    return RobotProtocol::MoveDirection::Right;
  }
  return RobotProtocol::MoveDirection::Unknown;
}

const char *CommandParser::directionToText(RobotProtocol::MoveDirection direction) const {
  switch (direction) {
    case RobotProtocol::MoveDirection::Forward:
      return "F";
    case RobotProtocol::MoveDirection::Back:
      return "B";
    case RobotProtocol::MoveDirection::Left:
      return "L";
    case RobotProtocol::MoveDirection::Right:
      return "R";
    case RobotProtocol::MoveDirection::Unknown:
    default:
      return "?";
  }
}
