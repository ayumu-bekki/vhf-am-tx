#include "si5351a.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <rom/ets_sys.h>

#include <cmath>
#include <memory>

#include "hardware_config.h"
#include "i2c_util.h"
#include "logger.h"

namespace transmitter_system {

namespace {
// Bus probe timeout when checking the synthesiser is actually present.
constexpr int kProbeTimeoutMs = 100;
}  // namespace

Si5351A::Si5351A()
    : dev_handle_(nullptr),
      i2c_address_(0),
      clk_output_status_(0),
      carrier_enabled_(false),
      frequency_set_(false),
      current_freq_hz_(0),
      carrier_clk_(Clk::kClk0) {}

Si5351A::~Si5351A() {
  if (dev_handle_) {
    i2c_master_bus_rm_device(dev_handle_);
    dev_handle_ = nullptr;
  }
}

bool Si5351A::Setup(const uint8_t address) {
  i2c_address_ = address;

  // Get the I2C bus handle
  i2c_master_bus_handle_t bus_handle = i2c_util::GetBusHandle();
  if (!bus_handle) {
    ESP_LOGE(kTag, "I2C bus not initialized");
    return false;
  }

  // Add device to the bus
  i2c_device_config_t dev_config = {};
  dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev_config.device_address = address;
  dev_config.scl_speed_hz = i2c_util::kI2cMasterFrequencyHz;

  esp_err_t ret =
      i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(kTag, "Failed to add Si5351A device: %s", esp_err_to_name(ret));
    dev_handle_ = nullptr;
    return false;
  }

  // Same reasoning as the display: probe before assuming the part is there,
  // so "not fitted" is distinguishable from "misbehaving".
  if (i2c_master_probe(bus_handle, address, kProbeTimeoutMs) != ESP_OK) {
    ESP_LOGE(kTag, "Si5351A not responding at address 0x%02X", address);
    i2c_master_bus_rm_device(dev_handle_);
    dev_handle_ = nullptr;
    return false;
  }

  clk_output_status_ = kAllOutputDisabled;

  ESP_LOGI(kTag, "Si5351A initializing at address 0x%02X", address);
  bool ok = true;
  ok &= Write(kRegOutputCtrl, clk_output_status_);  // 全出力を無効化
  ok &= Write(kRegClk0Ctrl, kClkPowerDown);         // CLK0をパワーダウン
  ok &= Write(kRegClk1Ctrl, kClkPowerDown);         // CLK1をパワーダウン
  ok &= Write(kRegClk2Ctrl, kClkPowerDown);         // CLK2をパワーダウン
  ok &= Write(kRegXtalLoad, kXtalLoad8pF);  // 水晶振動子の負荷容量を8pFに変更

  if (!ok) {
    ESP_LOGE(kTag, "Si5351A did not answer on the bus");
    return false;
  }

  ESP_LOGI(kTag, "Si5351A initialized");
  return true;
}

bool Si5351A::CarrierEnable() {
  if (!dev_handle_ || !frequency_set_) {
    ESP_LOGE(kTag, "CarrierEnable before frequency was programmed");
    return false;
  }
  if (carrier_enabled_) {
    return true;
  }

  const uint8_t clk_bit = 1 << static_cast<uint8_t>(carrier_clk_);
  const uint8_t reset_pll_bit =
      (carrier_clk_ == Clk::kClk0 ? kResetPllA : kResetPllB);

  // Reset the PLL so it reacquires cleanly, then release the output.
  if (!Write(kRegPllReset, reset_pll_bit)) {
    return false;
  }

  // Spec 8.2 step 1: give the PLL time to lock. The duty is still 0 here, so
  // the drain sits at 0V and nothing goes out while we wait.
  vTaskDelay(pdMS_TO_TICKS(hardware_config::kPllLockWaitMs));

  clk_output_status_ &= static_cast<uint8_t>(~clk_bit);
  if (!Write(kRegOutputCtrl, clk_output_status_)) {
    // Roll the shadow register back so a retry does not think it succeeded.
    clk_output_status_ |= clk_bit;
    return false;
  }

  carrier_enabled_ = true;
  ESP_LOGD(kTag, "Carrier enabled (%u Hz)", current_freq_hz_);
  return true;
}

bool Si5351A::CarrierDisable() {
  if (!dev_handle_) {
    return false;
  }
  if (!carrier_enabled_) {
    return true;
  }

  const uint8_t clk_bit = 1 << static_cast<uint8_t>(carrier_clk_);

  // Mark the carrier down before the write. If the bus write fails there is
  // nothing else we can do about it, and reporting "still transmitting" would
  // stall the stop sequence; the fault path handles a wedged bus.
  carrier_enabled_ = false;

  clk_output_status_ |= clk_bit;
  if (!Write(kRegOutputCtrl, clk_output_status_)) {
    ESP_LOGE(kTag, "Failed to disable carrier");
    return false;
  }

  ESP_LOGD(kTag, "Carrier disabled");
  return true;
}

bool Si5351A::SetFrequency(const Clk clk, const uint32_t freq) {
  // Nothing on the bus: fail quietly. Setup() already reported the absence
  // once, and the UI retunes on a timer, so logging per register write would
  // bury every other message.
  if (!dev_handle_) {
    return false;
  }

  const uint32_t denom = kPllDenom;

  // R分周器の自動選択
  // 低周波数出力時、MultiSynth分周比がkMultiSynthDivMaxを超える場合は、
  // R分周器(出力段の追加分周器)を使用して分周比を下げる
  uint32_t rdiv = 1;
  uint8_t rdiv_bits = 0;
  uint32_t target_freq = freq;

  while ((kPllFreqMin / target_freq) > kMultiSynthDivMax &&
         rdiv_bits < kRDivBitsMax) {
    rdiv <<= 1;
    rdiv_bits++;
    target_freq = freq * rdiv;
  }

  // R分周器を最大まで使っても分周比が範囲外の場合はエラー
  if ((kPllFreqMin / target_freq) > kMultiSynthDivMax) {
    ESP_LOGE(kTag, "Frequency %u Hz is too low (min: ~2.5kHz)", freq);
    return false;
  }

  // 最適なMultiSynth分周比とPLL周波数を計算
  uint32_t best_multisynth_div = 0;
  uint32_t best_pll_freq = 0;
  uint32_t min_error = UINT32_MAX;

  // 高周波数では偶数のみ試す（高速化）、低周波数では全て試す
  const uint32_t step = (target_freq >= kEvenDivOnlyThreshold) ? 2 : 1;

  for (uint32_t div = kMultiSynthDivMin; div <= kMultiSynthDivMax;
       div += step) {
    const uint32_t pll_freq = div * target_freq;

    if (pll_freq >= kPllFreqMin && pll_freq <= kPllFreqMax) {
      const uint32_t actual_freq = pll_freq / div / rdiv;
      const uint32_t error =
          (actual_freq > freq) ? (actual_freq - freq) : (freq - actual_freq);

      if (error < min_error) {
        min_error = error;
        best_multisynth_div = div;
        best_pll_freq = pll_freq;
      }

      if (error == 0) {
        break;
      }
    }
  }

  if (best_multisynth_div == 0) {
    ESP_LOGE(kTag, "Cannot find valid divider for freq: %u", freq);
    return false;
  }

  const uint32_t multisynth_div = best_multisynth_div;
  const uint32_t pll_freq = best_pll_freq;

  ESP_LOGD(kTag, "Freq: %u Hz, PLL: %u Hz, MS_div: %u, R_div: %u (bits: %u)",
           freq, pll_freq, multisynth_div, rdiv, rdiv_bits);

  // PLL分数分周器のパラメータ (a + b/c 形式)
  // 整数部 a
  const uint8_t pll_mult = pll_freq / kXtalFreq;
  // 分子 b: 整数演算で最近傍丸め (lround で切り捨てより誤差半減)
  const uint32_t pll_num = static_cast<uint32_t>(
      lround(static_cast<double>(pll_freq % kXtalFreq) * denom / kXtalFreq));

  // レジスタアドレスの決定
  const uint8_t reg_clk_ctrl = kRegClk0Ctrl + static_cast<uint8_t>(clk);
  const uint8_t reg_multisynth =
      kRegMs0 + static_cast<uint8_t>(clk) * kMultiSynthRegStride;
  const uint8_t reg_pll = (clk == Clk::kClk0 ? kRegPllA : kRegPllB);

  // 対象Clk出力を無効化
  clk_output_status_ |= (1 << static_cast<uint8_t>(clk));
  Write(kRegOutputCtrl, clk_output_status_);

  // Clkをパワーダウン
  Write(reg_clk_ctrl, kClkPowerDown);

  // PLL周波数を設定
  SetPLL(reg_pll, pll_mult, pll_num, denom);

  // Yield to let IDLE task feed WDT between bulk register writes
  taskYIELD();

  // MultiSynth分周器とR分周器を設定
  SetMultiSynth(reg_multisynth, multisynth_div, rdiv_bits);

  // Clkをパワーアップ (駆動電流: 2mA)
  const bool ok = Write(reg_clk_ctrl, kClkPower2mA);

  // The output stays disabled on purpose. Retuning while transmitting would
  // otherwise drop the carrier for the duration of the register writes, so
  // the transmit sequence (spec 8.2) decides when it comes up.
  carrier_clk_ = clk;
  current_freq_hz_ = freq;
  frequency_set_ = ok;

  // Retuning invalidates a running carrier; the caller must re-enable it.
  carrier_enabled_ = false;

  return ok;
}

bool Si5351A::Write(const uint8_t reg, const uint8_t value) {
  if (!dev_handle_) {
    ESP_LOGE(kTag, "Device handle not initialized");
    return false;
  }

  uint8_t write_buf[2] = {reg, value};
  esp_err_t ret = i2c_master_transmit(dev_handle_, write_buf, sizeof(write_buf),
                                      i2c_util::kI2cTimeoutMs);
  if (ret != ESP_OK) {
    ESP_LOGE(kTag, "Si5351A write failed (reg=0x%02X, val=0x%02X): %s", reg,
             value, esp_err_to_name(ret));
    return false;
  }
  return true;
}

// PLL分数分周器の設定
// PLL出力周波数 = 水晶周波数 × (mult + num/denom)
// 分数分周パラメータをSi5351Aレジスタ形式(P1, P2, P3)に変換して設定
void Si5351A::SetPLL(const uint8_t pll, const uint8_t mult, const uint32_t num,
                     const uint32_t denom) {
  ESP_LOGD(kTag, "SetPLL: mult=%u, num=%u, denom=%u (%.6f MHz)", mult, num,
           denom, kXtalFreq * (mult + (double)num / denom) / 1e6);

  // レジスタ形式への変換（Si5351Aデータシート参照）
  // 整数演算のみで計算 (128 * num の最大値 = 128 * 0xFFFFF ≈ 1.34e8 < 2^32)
  const uint32_t floor_128_num_denom = (128 * num) / denom;
  const uint32_t P1 = (128 * mult) + floor_128_num_denom - kP1Offset;
  const uint32_t P2 = (128 * num) - (denom * floor_128_num_denom);
  const uint32_t P3 = denom;

  ESP_LOGD(kTag, "PLL Registers: P1=%u, P2=%u, P3=%u", P1, P2, P3);

  // レジスタに書き込み
  Write(pll + 0, (P3 & 0x0000ff00) >> 8);
  Write(pll + 1, (P3 & 0x000000ff));
  Write(pll + 2, (P1 & 0x00030000) >> 16);
  Write(pll + 3, (P1 & 0x0000ff00) >> 8);
  Write(pll + 4, (P1 & 0x000000ff));
  Write(pll + 5, ((P3 & 0x000f0000) >> 12) | ((P2 & 0x000f0000) >> 16));
  Write(pll + 6, (P2 & 0x0000ff00) >> 8);
  Write(pll + 7, (P2 & 0x000000ff));
}

// MultiSynth分周器の設定
// 整数分周モード(P2=0, P3=1)で設定
// 最終出力周波数 = PLL周波数 ÷ div ÷ R分周器
void Si5351A::SetMultiSynth(const uint8_t synth, const uint32_t div,
                            const uint8_t rdiv_bits) {
  ESP_LOGD(kTag, "SetMultiSynth: div=%u, rdiv_bits=%u", div, rdiv_bits);

  // 整数モードの通常設定
  // P1 = 128 * a - 512 (a = 分周比)
  // P2 = 0 (整数モードでは常に0)
  // P3 = 1 (整数モードでは常に1)
  const uint32_t P1 = (128 * div) - kP1Offset;
  const uint32_t P2 = 0;
  const uint32_t P3 = 1;

  // DIVBY4設定 (出力周波数 > 150MHzの場合に必要)
  // MSx_DIVBY4[1:0] = 11b (bit 3-2 of register synth+2)
  uint8_t divby4_bits = 0;
  if (div == 4) {
    divby4_bits = kDivBy4Bits;
    ESP_LOGD(kTag, "Using DIVBY4 mode for div=4");
  }

  ESP_LOGD(kTag, "MS Registers: P1=%u, P2=%u, P3=%u, DIVBY4=0x%02X", P1, P2, P3,
           divby4_bits);

  // レジスタに書き込み
  Write(synth + 0, (P3 & 0x0000ff00) >> 8);
  Write(synth + 1, (P3 & 0x000000ff));
  // synth + 2 レジスタには R分周器(bit 6-4)、DIVBY4(bit
  // 3-2)、P1の上位ビット(bit 1-0)が含まれる
  Write(synth + 2,
        ((rdiv_bits & 0x07) << 4) | divby4_bits | ((P1 & 0x00030000) >> 16));
  Write(synth + 3, (P1 & 0x0000ff00) >> 8);
  Write(synth + 4, (P1 & 0x000000ff));
  Write(synth + 5, ((P3 & 0x000f0000) >> 12) | ((P2 & 0x000f0000) >> 16));
  Write(synth + 6, (P2 & 0x0000ff00) >> 8);
  Write(synth + 7, (P2 & 0x000000ff));
}

}  // namespace transmitter_system
