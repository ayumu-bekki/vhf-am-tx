#ifndef AM_TRANSMITTER_MAIN_TRANSMITTER_SYSTEM_H_
#define AM_TRANSMITTER_MAIN_TRANSMITTER_SYSTEM_H_
// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp

#include <esp_attr.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

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
///
/// Start() never returns: after bringing everything up, the calling task
/// (app_main) becomes the main loop - TX sequencing, the ARDF schedule,
/// battery and master tick supervision - under the task watchdog.
class TransmitterSystem final {
 public:
  TransmitterSystem();

  TransmitterSystem(const TransmitterSystem&) = delete;
  TransmitterSystem& operator=(const TransmitterSystem&) = delete;

  [[noreturn]] void Start();

 private:
  bool InitializeHardware();
  void ConfigureUnusedGpio();
  void ConfigureStatusLed();
  void SetStatusLed(bool on);
  void UpdateStatusLed(int64_t now_ms);
  void StartButtons();
  [[noreturn]] void RunMainLoop();

  // Callbacks handed to the modulator. These run in ISR context on every
  // 25kHz tick, so they must live in IRAM: a flash cache miss here costs tens
  // of microseconds and would blow the 20us budget of spec 7.1 on its own.
  static float IRAM_ATTR CwKeyerSample(void* context);
  static float IRAM_ATTR VoiceSample(void* context);
  static float IRAM_ATTR ToneSample(void* context);

  // A1A: the keyer (ISR) asks for CLK0 to be switched; this task does the I2C
  // write (spec 10.3). The ISR only wakes the task.
  static void IRAM_ATTR ClkRequest(void* context, bool on);
  static void ClkTaskEntry(void* arg);
  void ClkTaskLoop();

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

  // Last level written to the board LED, so the pin is only touched on a
  // change.
  bool status_led_on_ = false;

  TaskHandle_t clk_task_ = nullptr;

  GpioInputWatchTask gpio_watch_task_;
  UiControllerTask ui_task_;
};

}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_TRANSMITTER_SYSTEM_H_
