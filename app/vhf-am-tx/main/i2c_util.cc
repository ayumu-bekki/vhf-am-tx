// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// Shared I2C master bus (Si5351A + ST7032)

// Include ----------------------
#include "i2c_util.h"

#include "logger.h"

namespace transmitter_system {
namespace i2c_util {

// Global I2C master bus handle
static i2c_master_bus_handle_t g_bus_handle = nullptr;

void InitializeMaster(const gpio_num_t sda_pin, const gpio_num_t scl_pin) {
  i2c_master_bus_config_t bus_config = {};
  bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_config.i2c_port = I2C_NUM_0;
  bus_config.scl_io_num = scl_pin;
  bus_config.sda_io_num = sda_pin;
  bus_config.glitch_ignore_cnt = 7;
  bus_config.flags.enable_internal_pullup = true;

  esp_err_t ret = i2c_new_master_bus(&bus_config, &g_bus_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(kTag, "Failed to initialize I2C master bus: %s",
             esp_err_to_name(ret));
  } else {
    ESP_LOGI(kTag, "I2C master bus initialized (%ukHz, SDA=%d, SCL=%d)",
             i2c_util::kI2cMasterFrequencyHz / 1000, sda_pin, scl_pin);

    // Recover I2C bus in case a device is holding SDA low from a previous
    // incomplete transaction (e.g. brownout reset mid-transfer).
    i2c_master_bus_reset(g_bus_handle);
  }
}

i2c_master_bus_handle_t GetBusHandle() { return g_bus_handle; }

}  // namespace i2c_util
}  // namespace transmitter_system

// EOF
