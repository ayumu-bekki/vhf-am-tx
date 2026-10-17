#ifndef AM_TRANSMITTER_MAIN_LOGGER_H_
#define AM_TRANSMITTER_MAIN_LOGGER_H_
// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp

// Include ----------------------
#include <esp_log.h>

namespace transmitter_system {

/// Application Log Tag
static constexpr char kTag[] = "AmTransmitter";

namespace logger {

/// Quietens the IDF components to WARN and lets this application log at INFO
/// (VERBOSE when the build's default level is VERBOSE).
void InitializeLogLevel();

}  // namespace logger
}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_LOGGER_H_
