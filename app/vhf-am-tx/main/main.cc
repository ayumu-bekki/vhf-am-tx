// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp

// Include ----------------------
#include "transmitter_system.h"

/// Entry Point
extern "C" void app_main() {
  std::make_shared<transmitter_system::TransmitterSystem>()->Start();
}
