// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// ST7032 character LCD (16x2)

#include "st7032.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <rom/ets_sys.h>

#include <memory>

#include "i2c_util.h"
#include "logger.h"

namespace transmitter_system {

namespace {
// The datasheet asks for 26.3us between instructions.
constexpr uint32_t kInstructionDelayUs = 27;
// Clear display needs considerably longer.
constexpr uint32_t kClearDelayUs = 2000;
// DDRAM row base addresses
constexpr uint8_t kRowOffsets[] = {0x00, 0x40, 0x14, 0x54};
// Bus probe timeout when checking the display is actually present.
constexpr int kProbeTimeoutMs = 100;
}  // namespace

ST7032::ST7032()
    : dev_handle_(nullptr),
      i2c_address_(0),
      display_function_(0),
      display_control_(0),
      display_mode_(0),
      num_lines_(0),
      num_cols_(0) {}

ST7032::~ST7032() {
  if (dev_handle_) {
    i2c_master_bus_rm_device(dev_handle_);
    dev_handle_ = nullptr;
  }
}

bool ST7032::Setup(const uint8_t address, const uint8_t cols,
                   const uint8_t lines, const uint8_t charsize) {
  i2c_address_ = address;
  num_cols_ = cols;
  num_lines_ = lines;

  i2c_master_bus_handle_t bus_handle = i2c_util::GetBusHandle();
  if (!bus_handle) {
    ESP_LOGE(kTag, "I2C bus not initialized");
    return false;
  }

  i2c_device_config_t dev_config = {};
  dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev_config.device_address = address;
  dev_config.scl_speed_hz = i2c_util::kI2cMasterFrequencyHz;

  esp_err_t ret =
      i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(kTag, "Failed to add ST7032 device: %s", esp_err_to_name(ret));
    dev_handle_ = nullptr;
    return false;
  }

  // Adding the device only registers an address; it does not touch the bus.
  // Probe so a missing or unpowered display is reported here rather than as a
  // stream of write failures later.
  if (i2c_master_probe(bus_handle, address, kProbeTimeoutMs) != ESP_OK) {
    ESP_LOGE(kTag, "ST7032 not responding at address 0x%02X", address);
    i2c_master_bus_rm_device(dev_handle_);
    dev_handle_ = nullptr;
    return false;
  }

  display_function_ = kLcd8bitmode | kLcd1line | kLcd5x8dots;
  if (lines > 1) {
    display_function_ |= kLcd2line;
  }
  if ((charsize != 0) && (lines == 1)) {
    display_function_ |= kLcd5x10dots;
  }

  // Wait for power on
  vTaskDelay(pdMS_TO_TICKS(40));

  NormalFunctionSet();

  ExtendFunctionSet();
  Command(kLcdExSetbiasosc | kLcdBias1_5 | kLcdOsc183hz);
  Command(kLcdExFollowercontrol | kLcdFollowerOn | kLcdRab2_00);
  vTaskDelay(pdMS_TO_TICKS(200));
  NormalFunctionSet();

  display_control_ = 0x00;
  SetDisplayControl(kLcdDisplayon | kLcdCursoroff | kLcdBlinkoff);

  Clear();

  display_mode_ = 0x00;
  SetEntryMode(kLcdEntryleft | kLcdEntryshiftdecrement);

  ESP_LOGI(kTag, "ST7032 initialized at address 0x%02X (%ux%u)", address, cols,
           lines);
  return true;
}

void ST7032::SetContrast(uint8_t contrast) {
  ExtendFunctionSet();
  Command(kLcdExContrastsetl | (contrast & 0x0f));
  Command(kLcdExPowiconcontrasth | kLcdIconOn | kLcdBoostOn |
          ((contrast >> 4) & 0x03));
  NormalFunctionSet();
}

void ST7032::Clear() {
  Command(kLcdClearDisplay);
  ets_delay_us(kClearDelayUs);
}

void ST7032::SetCursor(uint8_t col, uint8_t row) {
  if (row >= num_lines_) {
    row = num_lines_ > 0 ? num_lines_ - 1 : 0;
  }
  Command(kLcdSetDDRRamAddr | (col + kRowOffsets[row]));
}

void ST7032::SetCursorVisible(bool cursor_on, bool blink_on) {
  // Rebuild the control byte rather than OR-ing, so turning the cursor back
  // off actually works. The B-Fox helper only ever sets bits.
  display_control_ =
      kLcdDisplayon | (cursor_on ? kLcdCursoron : kLcdCursoroff) |
      (blink_on ? kLcdBlinkon : kLcdBlinkoff);
  Command(kLcdDisplayControl | display_control_);
}

void ST7032::Print(const std::string& s) {
  Write(reinterpret_cast<const uint8_t*>(s.c_str()), s.length());
}

void ST7032::Print(const char str[]) {
  const size_t len = strlen(str);
  Write(reinterpret_cast<const uint8_t*>(str), len);
}

void ST7032::Printf(const char* format, ...) {
  va_list arg;
  va_start(arg, format);
  Vprintf(format, arg);
  va_end(arg);
}

void ST7032::Vprintf(const char* format, va_list arg) {
  constexpr size_t kStackBufSize = 64;
  char stack_buf[kStackBufSize] = {};

  va_list copy;
  va_copy(copy, arg);
  const int len = vsnprintf(stack_buf, kStackBufSize, format, copy);
  va_end(copy);

  if (len < 0) {
    return;
  }

  if (static_cast<size_t>(len) < kStackBufSize) {
    Write(reinterpret_cast<uint8_t*>(stack_buf), len);
  } else {
    auto heap_buf = std::make_unique<char[]>(len + 1);
    vsnprintf(heap_buf.get(), len + 1, format, arg);
    Write(reinterpret_cast<uint8_t*>(heap_buf.get()), len);
  }
}

void ST7032::Command(const uint8_t value) {
  Transmit(kControlCommand, value);
}

void ST7032::Write(const uint8_t value) { Transmit(kControlData, value); }

void ST7032::Write(const uint8_t* buffer, size_t size) {
  while (size--) {
    Write(*buffer++);
  }
}

void ST7032::Transmit(const uint8_t control, const uint8_t value) {
  if (!dev_handle_) {
    return;
  }

  const uint8_t write_buf[2] = {control, value};
  esp_err_t ret = i2c_master_transmit(dev_handle_, write_buf, sizeof(write_buf),
                                      i2c_util::kI2cTimeoutMs);
  if (ret != ESP_OK) {
    // Sporadic NACKs happen with RF noise on the bus. Give the controller a
    // moment and try once more before reporting.
    ets_delay_us(kInstructionDelayUs * 4);
    ret = i2c_master_transmit(dev_handle_, write_buf, sizeof(write_buf),
                              i2c_util::kI2cTimeoutMs);
  }
  if (ret != ESP_OK) {
    ESP_LOGE(kTag, "ST7032 write failed (ctrl=0x%02X, val=0x%02X): %s", control,
             value, esp_err_to_name(ret));
  }

  ets_delay_us(kInstructionDelayUs);
}

void ST7032::SetDisplayControl(uint8_t set_bit) {
  display_control_ |= set_bit;
  Command(kLcdDisplayControl | display_control_);
}

void ST7032::SetEntryMode(uint8_t set_bit) {
  display_mode_ |= set_bit;
  Command(kLcdEntryModeSet | display_mode_);
}

void ST7032::NormalFunctionSet() {
  Command(kLcdFunctionSet | display_function_);
}

void ST7032::ExtendFunctionSet() {
  Command(kLcdFunctionSet | display_function_ | kLcdExInstruction);
}

}  // namespace transmitter_system

// EOF
