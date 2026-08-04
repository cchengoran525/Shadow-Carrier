#include "WebServerHandlers.h"

#include <Arduino.h>

#include "Config.h"
#include "Protocol.h"

static WebServerHandlers *activeHandlers = nullptr;

namespace {

String buildControlPage() {
  String page;
  page.reserve(3200);

  page += F("<!doctype html><html lang='en'><head>");
  page += F("<meta charset='utf-8'>");
  page += F("<meta name='viewport' content='width=device-width,initial-scale=1'>");
  page += F("<title>Distributed Robot Platform</title>");
  page += F("<style>");
  page += F("body{margin:0;font-family:Arial,sans-serif;background:#f4f7fb;color:#1d2733;}");
  page += F("main{max-width:520px;margin:0 auto;padding:24px;}");
  page += F("h1{font-size:24px;margin:0 0 8px;}");
  page += F("p{margin:0 0 20px;color:#53606f;}");
  page += F(".pad{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;}");
  page += F("button{width:100%;height:72px;border:0;border-radius:8px;background:#1f6feb;color:white;font-size:18px;font-weight:700;touch-action:none;user-select:none;}");
  page += F("button.stop{background:#d1242f;}");
  page += F("button:active{transform:scale(.98);filter:brightness(.92);}");
  page += F(".empty{height:72px;}");
  page += F("#status{margin-top:18px;padding:12px;border-radius:8px;background:white;color:#334155;}");
  page += F("</style></head><body><main>");
  page += F("<h1>Robot Gateway</h1>");
  page += F("<p>ESP32-S3 HTTP to UART control panel</p>");
  page += F("<div class='pad'>");
  page += F("<div class='empty'></div><button data-path='/forward' data-label='Forward'>Forward</button><div class='empty'></div>");
  page += F("<button data-path='/left' data-label='Left'>Left</button><button class='stop' data-stop='1'>Stop</button><button data-path='/right' data-label='Right'>Right</button>");
  page += F("<div class='empty'></div><button data-path='/back' data-label='Back'>Back</button><div class='empty'></div>");
  page += F("</div>");
  page += F("<div id='status'>Ready</div>");
  page += F("<script>");
  page += F("let holdTimer=null,activeButton=null,activePointerId=null;");
  page += F("async function sendCmd(path,label){");
  page += F("const s=document.getElementById('status');s.textContent='Sending '+label+'...';");
  page += F("try{const r=await fetch(path,{method:'POST'});s.textContent=await r.text();}");
  page += F("catch(e){s.textContent='Request failed: '+e;}");
  page += F("}");
  page += F("function clearHold(sendStop){if(holdTimer){clearInterval(holdTimer);holdTimer=null;}activeButton=null;activePointerId=null;if(sendStop)sendCmd('/stop','Stop');}");
  page += F("function startHold(b,e){clearHold(false);activeButton=b;activePointerId=e.pointerId;if(b.setPointerCapture)b.setPointerCapture(e.pointerId);const tick=()=>sendCmd(b.dataset.path,b.dataset.label);tick();holdTimer=setInterval(tick,150);}");
  page += F("function stopHold(){clearHold(true);}");
  page += F("document.querySelectorAll('button').forEach(b=>{");
  page += F("b.addEventListener('contextmenu',e=>e.preventDefault());");
  page += F("if(b.dataset.stop){b.addEventListener('pointerdown',e=>{e.preventDefault();stopHold();});return;}");
  page += F("b.addEventListener('pointerdown',e=>{e.preventDefault();startHold(b,e);});");
  page += F("b.addEventListener('pointerup',e=>{e.preventDefault();if(activeButton===b)stopHold();});");
  page += F("b.addEventListener('pointercancel',e=>{if(activeButton===b)stopHold();});");
  page += F("b.addEventListener('lostpointercapture',()=>{if(activeButton===b)stopHold();});");
  page += F("});");
  page += F("window.addEventListener('pointerup',()=>{if(activeButton)stopHold();});");
  page += F("window.addEventListener('blur',()=>{if(activeButton)stopHold();});");
  page += F("document.addEventListener('visibilitychange',()=>{if(document.hidden&&activeButton)stopHold();});");
  page += F("</script></main></body></html>");

  return page;
}

void requireActiveHandlers() {
  if (activeHandlers == nullptr) {
    Serial.println("Web server handler is not initialized.");
  }
}

}  // namespace

WebServerHandlers::WebServerHandlers(UartGateway &gateway)
    : gateway_(gateway), server_(HTTP_SERVER_PORT) {}

void WebServerHandlers::begin() {
  server_.on("/", HTTP_GET, [this]() { handleRoot(); });
  server_.on("/forward", HTTP_POST, [this]() { handleForward(); });
  server_.on("/back", HTTP_POST, [this]() { handleBack(); });
  server_.on("/left", HTTP_POST, [this]() { handleLeft(); });
  server_.on("/right", HTTP_POST, [this]() { handleRight(); });
  server_.on("/stop", HTTP_POST, [this]() { handleStop(); });
  server_.on("/ping", HTTP_POST, [this]() { handlePing(); });
  server_.onNotFound([this]() { handleNotFound(); });

  server_.begin();
  Serial.println("HTTP server started.");
}

void WebServerHandlers::handleClient() {
  server_.handleClient();
}

void WebServerHandlers::handleRoot() {
  server_.send(200, "text/html", buildControlPage());
}

void WebServerHandlers::handleForward() {
  sendRobotCommand(RobotProtocol::COMMAND_FORWARD, "Forward");
}

void WebServerHandlers::handleBack() {
  sendRobotCommand(RobotProtocol::COMMAND_BACK, "Back");
}

void WebServerHandlers::handleLeft() {
  sendRobotCommand(RobotProtocol::COMMAND_LEFT, "Left");
}

void WebServerHandlers::handleRight() {
  sendRobotCommand(RobotProtocol::COMMAND_RIGHT, "Right");
}

void WebServerHandlers::handleStop() {
  sendRobotCommand(RobotProtocol::COMMAND_STOP, "Stop");
}

void WebServerHandlers::handlePing() {
  sendRobotCommand(RobotProtocol::COMMAND_PING, "Ping");
}

void WebServerHandlers::handleNotFound() {
  server_.send(404, "text/plain", "Not found");
}

void WebServerHandlers::sendRobotCommand(const char *command, const char *label) {
  gateway_.sendCommand(command);

  String response = String(label) + " command sent: " + command;
  server_.send(200, "text/plain", response);
}

void initWebServer(WebServerHandlers &handlers) {
  activeHandlers = &handlers;
  activeHandlers->begin();
}

void handleWebClient() {
  requireActiveHandlers();
  if (activeHandlers != nullptr) {
    activeHandlers->handleClient();
  }
}
