#ifndef AM_TRANSMITTER_MAIN_LOGGER_H_
#define AM_TRANSMITTER_MAIN_LOGGER_H_
// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp

// Include ----------------------
#include <esp_log.h>

// Enable all log levels up to Verbose at compile time
#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL CONFIG_LOG_DEFAULT_LEVEL

namespace transmitter_system {

/// Application Log Tag
static constexpr char kTag[] = "AmTransmitter";

namespace logger {

void InitializeLogLevel();

}  // namespace logger
}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_LOGGER_H_
