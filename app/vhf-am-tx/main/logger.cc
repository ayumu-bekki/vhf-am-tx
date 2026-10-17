// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp

// Include ----------------------
#include "logger.h"

namespace transmitter_system {
namespace logger {

void InitializeLogLevel() {
  esp_log_level_set("*", ESP_LOG_WARN);
  esp_log_level_set(kTag, CONFIG_LOG_DEFAULT_LEVEL >= ESP_LOG_VERBOSE
                              ? ESP_LOG_VERBOSE
                              : ESP_LOG_INFO);
}

}  // namespace logger
}  // namespace transmitter_system
