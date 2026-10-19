#ifndef AM_TRANSMITTER_MAIN_SI5351A_H_
#define AM_TRANSMITTER_MAIN_SI5351A_H_
// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// Si5351A clock generator (carrier source, CLK0)

#include <driver/i2c_master.h>

#include <cstdint>
#include <mutex>

namespace transmitter_system {

/// Si5351A driver.
///
/// Called from two tasks: the UI retunes it, the main loop (TxController)
/// switches the carrier on and off. Every public operation takes mutex_, so
/// a multi-register sequence and the shadow of the output enable register
/// can never interleave.
class Si5351A {
 public:
  static constexpr uint8_t kI2cDefaultAddr = 0x60;

  // 周波数設定
  static constexpr uint32_t kXtalFreq = 25000000;     // 水晶発振器: 25MHz
  static constexpr uint32_t kPllFreqMin = 600000000;  // PLL周波数最小: 600MHz
  static constexpr uint32_t kPllFreqMax = 900000000;  // PLL周波数最大: 900MHz

  // MultiSynth分周比の制約
  // 整数モード: 6～2048 (レジスタ制限)
  // 分数モード: 8 + 1/1048575 ～ 900
  static constexpr uint32_t kMultiSynthDivMin = 4;
  static constexpr uint32_t kMultiSynthDivMax = 2048;

  // R分周器の最大値 (1, 2, 4, 8, 16, 32, 64, 128)
  static constexpr uint8_t kRDivMax = 0x80;
  static constexpr uint8_t kRDivBitsMax =
      7;  // R分周器のレジスタ値の最大 (2^7 = 128)

  // 分周比探索の閾値
  // この周波数以上の場合、偶数分周比のみを試す（計算高速化のため）
  static constexpr uint32_t kEvenDivOnlyThreshold = 100000;  // 100kHz

  // レジスタアドレス
  static constexpr uint8_t kRegOutputCtrl = 0x03;
  static constexpr uint8_t kRegXtalLoad = 0xb7;
  static constexpr uint8_t kRegClk0Ctrl = 0x10;
  static constexpr uint8_t kRegClk1Ctrl = 0x11;
  static constexpr uint8_t kRegClk2Ctrl = 0x12;
  static constexpr uint8_t kRegPllA = 0x1a;
  static constexpr uint8_t kRegPllB = 0x22;
  static constexpr uint8_t kRegMs0 = 0x2a;
  static constexpr uint8_t kRegMs1 = 0x32;
  static constexpr uint8_t kRegMs2 = 0x3a;
  static constexpr uint8_t kRegPllReset = 0xb1;

  // CLK Control Register values (Integer mode enabled)
  // bit 7: Power Down (0 = enabled, 1 = disabled)
  // bit 6: MS Integer Mode (1 = integer, 0 = fractional)
  // bit 5: MS Source (0 = PLLA, 1 = PLLB)
  // bit 4: CLK Invert (0 = not inverted, 1 = inverted)
  // bit 3-2: CLK_SRC (11 = MS_SRC, 10 = MS_SRC, 01 = CLKIN, 00 = XTAL)
  // bit 1-0: CLK_IDRV Drive Strength (11=8mA, 10=6mA, 01=4mA, 00=2mA)
  // 0x4F = 0b01001111: Power Up, Integer Mode, PLLA, Not Inverted, MultiSynth,
  // 8mA
  static constexpr uint8_t kClkPower8mA =
      0x4F;  // Integer mode + PLLA + MultiSynth + 8mA
  static constexpr uint8_t kClkPower6mA =
      0x4E;  // Integer mode + PLLA + MultiSynth + 6mA
  static constexpr uint8_t kClkPower4mA =
      0x4D;  // Integer mode + PLLA + MultiSynth + 4mA
  static constexpr uint8_t kClkPower2mA =
      0x4C;  // Integer mode + PLLA + MultiSynth + 2mA

  static constexpr uint8_t kResetPllA = 0x20;
  static constexpr uint8_t kResetPllB = 0x80;

  // PLL分数分周器の分母 (20bit最大分解能)
  static constexpr uint32_t kPllDenom = 0x000fffff;

  // MultiSynth レジスタアドレスオフセット (CLKごとに8バイト)
  static constexpr uint8_t kMultiSynthRegStride = 0x08;

  // P1 計算オフセット: P1 = 128*a + floor(128*b/c) - 512
  static constexpr uint32_t kP1Offset = 512;

  // DIVBY4 制御ビット: MSx_DIVBY4[1:0] = 11b (bit 3-2)
  static constexpr uint8_t kDivBy4Bits = 0x0C;

  // 全出力無効化マスク
  static constexpr uint8_t kAllOutputDisabled = 0xFF;

  // CLK パワーダウンビット
  static constexpr uint8_t kClkPowerDown = 0x80;

  // 水晶の負荷容量 (レジスタ 0xB7, AN619)
  // bit 7-6: XTAL_CL (01 = 6pF, 10 = 8pF, 11 = 10pF)
  // bit 5-0: 予約。データシートの指定どおり 010010b (0x12) を書く。
  // 水晶の仕様(負荷容量)に合わせて選ぶこと。
  static constexpr uint8_t kXtalLoadReserved = 0x12;
  static constexpr uint8_t kXtalLoad6pF = 0x40 | kXtalLoadReserved;   // 0x52
  static constexpr uint8_t kXtalLoad8pF = 0x80 | kXtalLoadReserved;   // 0x92
  static constexpr uint8_t kXtalLoad10pF = 0xC0 | kXtalLoadReserved;  // 0xD2

  enum class Clk {
    kClk0 = 0,
    kClk1,
    kClk2,
  };

 public:
  Si5351A();
  ~Si5351A();

  bool Setup(const uint8_t address);

  /// Programs the PLL and MultiSynth for the requested output frequency.
  ///
  /// Unlike the receiver version this leaves the output DISABLED. On a
  /// transmitter the carrier must only appear when the transmit sequence says
  /// so (spec 8.2), so tuning and keying are deliberately separate: call
  /// CarrierEnable() afterwards.
  bool SetFrequency(const Clk clk, const uint32_t freq);

  /// Enables CLK0 and waits for the PLL to lock (spec 8.2 step 1).
  /// Duty must still be 0 at this point - the drain is at 0V so nothing
  /// radiates while the PLL settles.
  bool CarrierEnable();

  /// Disables CLK0 (spec 8.3 step 3). Must only be called after the duty has
  /// been ramped down and the drain has settled, never before.
  bool CarrierDisable();

  /// Switches only the CLK0 output on or off, leaving the PLL running (A1A,
  /// spec 10.3): one I2C write, no lock wait. No-op unless the carrier has
  /// been enabled by CarrierEnable().
  bool CarrierOutput(bool on);

  bool IsReady() const { return dev_handle_ != nullptr; }

 private:
  bool Write(const uint8_t reg, const uint8_t value);

  bool SetPLL(const uint8_t pll, const uint8_t mult, const uint32_t num,
              const uint32_t denom);
  bool SetMultiSynth(const uint8_t synth, const uint32_t div,
                     const uint8_t rdiv_bits);

 private:
  std::mutex mutex_;
  i2c_master_dev_handle_t dev_handle_;

  uint8_t clk_output_status_;

  bool carrier_enabled_;
  bool frequency_set_;
  uint32_t current_freq_hz_;
  Clk carrier_clk_;
};

}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_SI5351A_H_
