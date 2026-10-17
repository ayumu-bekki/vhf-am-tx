#ifndef AM_TRANSMITTER_MAIN_UTIL_H_
#define AM_TRANSMITTER_MAIN_UTIL_H_
// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp

// Include ----------------------
#include <chrono>
#include <string>
#include <vector>

#include <esp_err.h>

namespace transmitter_system {
namespace util {

/// Sleep
void SleepMillisecond(const uint32_t sleep_milliseconds);

/// Split Text
std::vector<std::string> SplitString(const std::string& str, const char delim);

/// Initialize NVS flash (call once at startup)
bool InitializeNvs();

}  // namespace util
}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_UTIL_H_
