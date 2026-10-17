#ifndef AM_TRANSMITTER_MAIN_TRANSMITTER_SYSTEM_H_
#define AM_TRANSMITTER_MAIN_TRANSMITTER_SYSTEM_H_
// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp

#include <esp_attr.h>

#include <memory>

#include "adc_sampler.h"
#include "ardf_scheduler.h"
#include "cw_keyer.h"
#include "gpio_input_watch_task.h"
#include "modulator.h"
#include "si5351a.h"
#include "st7032.h"
#include "tone_generator.h"
#include "transmitter_settings.h"
#include "tx_controller.h"
#include "ui_controller_task.h"
#include "voice_dsp.h"

namespace transmitter_system {

/// Owns every subsystem and wires them together.
class TransmitterSystem final
    : public std::enable_shared_from_this<TransmitterSystem> {
 public:
  TransmitterSystem();
  ~TransmitterSystem();

  void Start();

 private:
  bool InitializeHardware();
  void ConfigureUnusedGpio();
  void ConfigureStatusLed();
  void SetStatusLed(bool on);
  void RunMainLoop();

  // Callbacks handed to the modulator. These run in ISR context on every
  // 25kHz tick, so they must live in IRAM: a flash cache miss here costs tens
  // of microseconds and would blow the 20us budget of spec 7.1 on its own.
  static float IRAM_ATTR CwKeyerSample(void* context);
  static float IRAM_ATTR VoiceSample(void* context);
  static float IRAM_ATTR ToneSample(void* context);

 private:
  Si5351A si5351a_;
  ST7032 lcd_;
  AdcSampler adc_;
  TransmitterSettings settings_;

  ToneGenerator tone_;
  CwKeyer keyer_;
  VoiceDsp voice_;
  Modulator modulator_;

  TxController tx_;
  ArdfScheduler ardf_;

  // False when the LCD did not answer at start-up; the board LED blinks.
  bool lcd_ok_ = true;

  GpioInputWatchTask gpio_watch_task_;
  UiControllerTaskUniquePtr ui_task_;
};

}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_TRANSMITTER_SYSTEM_H_
