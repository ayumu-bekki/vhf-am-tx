// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp

// Include ----------------------
#include "transmitter_system.h"

/// Entry Point
extern "C" void app_main() {
  // Static storage: the system owns several kilobytes of tables and buffers,
  // and Start() never returns, so it lives for the life of the program.
  static transmitter_system::TransmitterSystem system;
  system.Start();
}
