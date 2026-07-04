#ifndef WEB_SERVER_HANDLERS_H
#define WEB_SERVER_HANDLERS_H

#include <WebServer.h>

#include "UartGateway.h"

class WebServerHandlers {
 public:
  explicit WebServerHandlers(UartGateway &gateway);

  void begin();
  void handleClient();

 private:
  void handleRoot();
  void handleForward();
  void handleBack();
  void handleLeft();
  void handleRight();
  void handleStop();
  void handlePing();
  void handleNotFound();
  void sendRobotCommand(const char *command, const char *label);

  UartGateway &gateway_;
  WebServer server_;
};

void initWebServer(WebServerHandlers &handlers);
void handleWebClient();

#endif
