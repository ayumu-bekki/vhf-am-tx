#ifndef AM_TRANSMITTER_MAIN_ST7032_H_
#define AM_TRANSMITTER_MAIN_ST7032_H_
// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// ST7032 character LCD (16x2)
//
// Ported from the B-Fox receiver driver. The register map and the
// initialisation sequence are unchanged; only the I2C transport moved from the
// legacy driver (i2c_cmd_link_*) to the i2c_master driver, because the legacy
// one is end-of-life in ESP-IDF v6.0 and cannot share a bus with the new one.

#include <driver/i2c_master.h>
#include <esp_log.h>

#include <cstdarg>
#include <cstring>
#include <memory>
#include <string>

namespace transmitter_system {

class ST7032 {
 public:
  static constexpr uint8_t kI2cDefaultAddr = 0x3E;

  static constexpr uint8_t kLcdClearDisplay = 0x01;
  static constexpr uint8_t kLcdReturnHome = 0x02;
  static constexpr uint8_t kLcdEntryModeSet = 0x04;
  static constexpr uint8_t kLcdDisplayControl = 0x08;
  static constexpr uint8_t kLcdCursorShift = 0x10;
  static constexpr uint8_t kLcdFunctionSet = 0x20;
  static constexpr uint8_t kLcdSetCGRamAddr = 0x40;
  static constexpr uint8_t kLcdSetDDRRamAddr = 0x80;

  // Extended instruction table (IS = 1)
  static constexpr uint8_t kLcdExSetbiasosc = 0x10;
  static constexpr uint8_t kLcdExSeticonramaddr = 0x40;
  static constexpr uint8_t kLcdExPowiconcontrasth = 0x50;
  static constexpr uint8_t kLcdExFollowercontrol = 0x60;
  static constexpr uint8_t kLcdExContrastsetl = 0x70;

  static constexpr uint8_t kLcdEntryright = 0x00;
  static constexpr uint8_t kLcdEntryleft = 0x02;
  static constexpr uint8_t kLcdEntryshiftincrement = 0x01;
  static constexpr uint8_t kLcdEntryshiftdecrement = 0x00;

  static constexpr uint8_t kLcdDisplayon = 0x04;
  static constexpr uint8_t kLcdDisplayoff = 0x00;
  static constexpr uint8_t kLcdCursoron = 0x02;
  static constexpr uint8_t kLcdCursoroff = 0x00;
  static constexpr uint8_t kLcdBlinkon = 0x01;
  static constexpr uint8_t kLcdBlinkoff = 0x00;

  static constexpr uint8_t kLcd8bitmode = 0x10;
  static constexpr uint8_t kLcd4bitmode = 0x00;
  static constexpr uint8_t kLcd2line = 0x08;
  static constexpr uint8_t kLcd1line = 0x00;
  static constexpr uint8_t kLcd5x10dots = 0x04;
  static constexpr uint8_t kLcd5x8dots = 0x00;
  static constexpr uint8_t kLcdExInstruction = 0x01;  // IS bit

  static constexpr uint8_t kLcdBias1_4 = 0x08;
  static constexpr uint8_t kLcdBias1_5 = 0x00;

  static constexpr uint8_t kLcdIconOn = 0x08;
  static constexpr uint8_t kLcdIconOff = 0x00;
  static constexpr uint8_t kLcdBoostOn = 0x04;
  static constexpr uint8_t kLcdBoostOff = 0x00;
  static constexpr uint8_t kLcdOsc122hz = 0x00;
  static constexpr uint8_t kLcdOsc131hz = 0x01;
  static constexpr uint8_t kLcdOsc144hz = 0x02;
  static constexpr uint8_t kLcdOsc161hz = 0x03;
  static constexpr uint8_t kLcdOsc183hz = 0x04;
  static constexpr uint8_t kLcdOsc221hz = 0x05;
  static constexpr uint8_t kLcdOsc274hz = 0x06;
  static constexpr uint8_t kLcdOsc347hz = 0x07;

  static constexpr uint8_t kLcdFollowerOn = 0x08;
  static constexpr uint8_t kLcdFollowerOff = 0x00;
  static constexpr uint8_t kLcdRab1_00 = 0x00;
  static constexpr uint8_t kLcdRab1_25 = 0x01;
  static constexpr uint8_t kLcdRab1_50 = 0x02;
  static constexpr uint8_t kLcdRab1_80 = 0x03;
  static constexpr uint8_t kLcdRab2_00 = 0x04;
  static constexpr uint8_t kLcdRab2_50 = 0x05;
  static constexpr uint8_t kLcdRab3_00 = 0x06;
  static constexpr uint8_t kLcdRab3_75 = 0x07;

  // Control bytes prefixed to every transfer
  static constexpr uint8_t kControlCommand = 0x00;
  static constexpr uint8_t kControlData = 0x40;

 public:
  ST7032();
  ~ST7032();

  /// Adds the device to the shared bus and runs the power-on sequence.
  /// Unlike the B-Fox version there is no i2c_port argument: the bus handle
  /// comes from i2c_util.
  bool Setup(const uint8_t address, const uint8_t cols, const uint8_t lines,
             const uint8_t charsize = kLcd5x8dots);

  void SetContrast(uint8_t contrast);
  void Clear();
  void SetCursor(uint8_t col, uint8_t row);
  void Print(const std::string& s);
  void Print(const char str[]);
  void Printf(const char* format, ...);
  void Vprintf(const char* format, va_list arg);

  /// Hardware cursor, used by the callsign editor (spec 9.4.4) to show which
  /// character is being edited.
  void SetCursorVisible(bool cursor_on, bool blink_on);

  void Write(const uint8_t* buffer, size_t size);
  void Write(const uint8_t value);
  void Command(const uint8_t value);

  bool IsReady() const { return dev_handle_ != nullptr; }

 private:
  void SetDisplayControl(uint8_t set_bit);
  void SetEntryMode(uint8_t set_bit);
  void NormalFunctionSet();
  void ExtendFunctionSet();
  void Transmit(const uint8_t control, const uint8_t value);

 private:
  i2c_master_dev_handle_t dev_handle_;
  uint8_t i2c_address_;

  uint8_t display_function_;
  uint8_t display_control_;
  uint8_t display_mode_;

  uint8_t num_lines_;
  uint8_t num_cols_;
};

using ST7032UniquePtr = std::unique_ptr<ST7032>;
using ST7032SharedPtr = std::shared_ptr<ST7032>;
using ST7032WeakPtr = std::weak_ptr<ST7032>;

}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_ST7032_H_
