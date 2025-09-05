#include <Arduino.h>
#include "UserConsole.hpp"

// Optional: fast, readonly status provider (no allocation)
static const char* SimpleStatus() {
    // Keep short and constant-time
    static const char kLine[] = "status: idle, axes: X0 Y0 Z0 (mock)";
    return kLine;
}

void setup() {
    using namespace cnc::console;

    // Console init (allocates only during startup)
    ConsoleConfig cfg;
    cfg.prompt = "CNC> ";
    cfg.baud = BaudRate::BR_115200;
    (void)UserConsole::Instance().Begin(cfg);
    UserConsole::Instance().SetStatusProvider(&SimpleStatus);

    // NOTE: Do not call Serial.begin(); ESP32Console owns stdio (see library docs).  // :contentReference[oaicite:6]{index=6}
}

void loop() {
    // Your realtime control tasks live elsewhere.
    // Keep loop() short; console runs in its own FreeRTOS task.
    delay(10);
}
