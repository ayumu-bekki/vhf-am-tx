#ifndef AM_TRANSMITTER_MAIN_I2C_UTIL_H_
#define AM_TRANSMITTER_MAIN_I2C_UTIL_H_
// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// Utilities

// Include ----------------------
#include <driver/gpio.h>
#include <driver/i2c_master.h>

#include "hardware_config.h"

namespace transmitter_system {
namespace i2c_util {

// I2C Master configuration constants
constexpr uint32_t kI2cMasterFrequencyHz = hardware_config::kI2cFrequencyHz;
constexpr int kI2cTimeoutMs = 50;  // 50ms timeout (prevent WDT starvation)

void InitializeMaster(const gpio_num_t sda_pin, const gpio_num_t scl_pin);
i2c_master_bus_handle_t GetBusHandle();

}  // namespace i2c_util
}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_I2C_UTIL_H_

// EOF
