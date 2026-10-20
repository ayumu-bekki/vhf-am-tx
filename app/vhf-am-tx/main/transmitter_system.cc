// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp

#include "transmitter_system.h"

#include <driver/gpio.h>
#include <esp_task_wdt.h>

#include <cstring>

#include "hardware_config.h"
#include "i2c_util.h"
#include "logger.h"
#include "util.h"
#include "version.h"
#include "xiao_esp32c6_pin.h"

namespace transmitter_system {

namespace {

/// Pins this design does not use. Parking them as inputs with a pull-down
/// keeps them from floating.
///
/// GPIO2 is deliberately absent: it is the PWM output, and it must stay
/// under LEDC control with its external pull-down as the only thing holding
/// it down during reset (spec 8.4).
const gpio_num_t kUnusedGpioPins[] = {
    xiao_esp32c6_pin::kD3,             // GPIO21 (former PWM output)
    xiao_esp32c6_pin::kWifiEnable,     // GPIO3
    xiao_esp32c6_pin::kMtms,           // GPIO4
    xiao_esp32c6_pin::kMtdi,           // GPIO5
    xiao_esp32c6_pin::kMtck,           // GPIO6
    xiao_esp32c6_pin::kMtdo,           // GPIO7
    xiao_esp32c6_pin::kBoot,           // GPIO9
    xiao_esp32c6_pin::kWifiAntConfig,  // GPIO14
};

}  // namespace

using util::NowMs;

TransmitterSystem::TransmitterSystem()
    : ui_task_(lcd_, settings_, tx_, ardf_, modulator_, si5351a_, keyer_) {}

float TransmitterSystem::CwKeyerSample(void* context) {
  return static_cast<CwKeyer*>(context)->Tick();
}

float TransmitterSystem::VoiceSample(void* context) {
  TransmitterSystem* self = static_cast<TransmitterSystem*>(context);
  return self->voice_.Process(self->adc_.ReadMicrophone());
}

float TransmitterSystem::ToneSample(void* context) {
  ToneGenerator* tone = static_cast<ToneGenerator*>(context);
  tone->SetKey(true);  // continuous, no keying
  return tone->Next();
}

void TransmitterSystem::ClkRequest(void* context, bool /*on*/) {
  TransmitterSystem* self = static_cast<TransmitterSystem*>(context);
  if (!self->clk_task_) {
    return;
  }
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(self->clk_task_, &woken);
  portYIELD_FROM_ISR(woken);
}

void TransmitterSystem::ClkTaskEntry(void* arg) {
  static_cast<TransmitterSystem*>(arg)->ClkTaskLoop();
}

// The state is read after the wake-up, not taken from the request: if two
// requests pile up the last one wins, and the register ends up right. The
// lead time is 1ms, so this task sits above every other one.
void TransmitterSystem::ClkTaskLoop() {
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    si5351a_.CarrierOutput(keyer_.WantsClkOutput());
  }
}

void TransmitterSystem::SetStatusLed(bool on) {
  const bool level = hardware_config::kStatusLedActiveLow ? !on : on;
  gpio_set_level(hardware_config::kStatusLedPin, level ? 1 : 0);
  status_led_on_ = on;
}

void TransmitterSystem::UpdateStatusLed(int64_t now_ms) {
  bool on = false;
  if (tx_.IsTransmitting()) {
    // Carrier up: solid. This wins over the LCD fault blink - whether the
    // set is radiating is the one thing the operator must always be able
    // to see.
    on = true;
  } else if (!lcd_ok_) {
    // The LCD is down: blink the board LED hard so the fault is noticed.
    on = (now_ms / hardware_config::kStatusLedBlinkMs) % 2 == 0;
  }
  if (on != status_led_on_) {
    SetStatusLed(on);
  }
}

void TransmitterSystem::ConfigureStatusLed() {
  gpio_reset_pin(hardware_config::kStatusLedPin);
  gpio_set_direction(hardware_config::kStatusLedPin, GPIO_MODE_OUTPUT);
  SetStatusLed(false);
}

void TransmitterSystem::ConfigureUnusedGpio() {
  for (const gpio_num_t gpio_num : kUnusedGpioPins) {
    gpio_set_direction(gpio_num, GPIO_MODE_INPUT);
    gpio_set_pull_mode(gpio_num, GPIO_PULLDOWN_ONLY);
  }
}

void TransmitterSystem::StartButtons() {
  MessageQueue<ButtonEvent>* const queue = ui_task_.GetButtonEventQueue();

  // SW_MODE is single shot; FN+/FN- auto-repeat (spec 9.7).
  using Button = GpioInputWatchTask::Button;
  gpio_watch_task_.AddButton(Button(hardware_config::kSwModePin,
                                    ButtonEvent::Button::kMode, queue,
                                    /*allow_repeat=*/false));
  gpio_watch_task_.AddButton(Button(hardware_config::kSwFnPlusPin,
                                    ButtonEvent::Button::kFnPlus, queue,
                                    /*allow_repeat=*/true));
  gpio_watch_task_.AddButton(Button(hardware_config::kSwFnMinusPin,
                                    ButtonEvent::Button::kFnMinus, queue,
                                    /*allow_repeat=*/true));
  gpio_watch_task_.Start();
}

bool TransmitterSystem::InitializeHardware() {
  const bool nvs_ok = util::InitializeNvs();
  settings_.Load();
  settings_.StartWriter();
  if (!nvs_ok || !settings_.IsStorageAvailable()) {
    ESP_LOGE(kTag, "Settings storage unavailable: nothing will be saved");
  }

  i2c_util::InitializeMaster(hardware_config::kI2cSdaPin,
                             hardware_config::kI2cSclPin);
  util::SleepMillisecond(hardware_config::kI2cBusStabilizeDelayMs);

  // The transmit sequence may only run once both I2C devices have answered
  // (spec 8.4).
  const bool bus_ok = i2c_util::GetBusHandle() != nullptr;
  if (!bus_ok) {
    ESP_LOGE(kTag, "I2C bus not available");
  }
  const bool si5351a_ok = si5351a_.Setup(Si5351A::kI2cDefaultAddr);
  const bool lcd_ok =
      lcd_.Setup(ST7032::kI2cDefaultAddr, hardware_config::kLcdColumns,
                 hardware_config::kLcdLines);
  if (lcd_ok) {
    lcd_.SetContrast(hardware_config::kLcdContrast);
    lcd_.Clear();
    // Same text the UI shows next, so the title is on screen from the moment
    // the display is up, not only once the UI task starts.
    lcd_.WriteAt(0, 0, hardware_config::kBootTitle,
                 std::strlen(hardware_config::kBootTitle));
    lcd_.WriteAt(0, 1, settings_.GetCallsign(),
                 std::strlen(settings_.GetCallsign()));  // blank if unset
  }

  // A missing synthesiser must not stop the modulator from coming up.
  //
  // The PWM path does not depend on the Si5351A at all, and the D:DUTY
  // calibration mode (spec 12) deliberately runs with the carrier disabled -
  // it is how DUTY_CARRIER gets trimmed against an oscilloscope, which is
  // step 7 of the bring-up, before the synthesiser is even in the picture.
  // Refusing to start here would make that impossible.
  //
  // Transmission stays blocked: TxController::RequestStart() fails when
  // CarrierEnable() does, so nothing can radiate without a working carrier.
  if (!si5351a_ok) {
    ESP_LOGW(kTag,
             "Si5351A absent: carrier disabled, modulator and D:DUTY still "
             "available");
  }
  if (!lcd_ok) {
    // Nothing to show anything on, so the board LED blinks fast instead
    // (RunMainLoop). The log has the details.
    ESP_LOGE(kTag, "LCD not available: board LED will blink");
    lcd_ok_ = false;
  }

  // Signal path. Tables first, then the LEDC output, then the tick.
  tone_.Initialize();
  voice_.Initialize();
  tone_.SetFrequencyHz(settings_.GetToneHz());
  ui_task_.SetToneGenerator(&tone_);
  keyer_.SetToneGenerator(&tone_);
  keyer_.SetWpm(settings_.GetCwWpm());

  constexpr UBaseType_t kClkTaskPriority = configMAX_PRIORITIES - 4;
  if (xTaskCreate(&TransmitterSystem::ClkTaskEntry, "clk_switch", 3072, this,
                  kClkTaskPriority, &clk_task_) != pdPASS) {
    clk_task_ = nullptr;
    ESP_LOGE(kTag, "CLK0 switching task failed: A1A unavailable");
  } else {
    keyer_.SetClkRequestCallback(&TransmitterSystem::ClkRequest, this);
  }

  if (!modulator_.Initialize()) {
    ESP_LOGE(kTag, "Modulator init failed");
    return false;
  }

  modulator_.SetCwKeyerCallback(&TransmitterSystem::CwKeyerSample, &keyer_);
  modulator_.SetVoiceCallback(&TransmitterSystem::VoiceSample, this);
  modulator_.SetToneCallback(&TransmitterSystem::ToneSample, &tone_);

  tx_.SetModulator(&modulator_);
  tx_.SetSi5351A(&si5351a_);
  tx_.SetToneGenerator(&tone_);
  tx_.SetCwKeyer(&keyer_);

  ardf_.SetTxController(&tx_);
  ardf_.SetCwKeyer(&keyer_);
  ardf_.SetArdfId(settings_.GetArdfId());
  ardf_.SetCallsign(settings_.GetCallsign());

  // The ADC free-runs; it is not the clock. Start it before the tick so the
  // bias measurement of spec 3.4 is already under way.
  if (!adc_.Initialize() || !adc_.Start()) {
    ESP_LOGE(kTag, "ADC init failed");
    return false;
  }

  if (!modulator_.Start()) {
    ESP_LOGE(kTag, "Master tick failed to start");
    return false;
  }

  return true;
}

void TransmitterSystem::Start() {
  logger::InitializeLogLevel();

  ESP_LOGI(kTag, "Startup 144MHz AM Transmitter. Version:%.*s",
           static_cast<int>(kGitVersion.size()), kGitVersion.data());

  ConfigureUnusedGpio();
  ConfigureStatusLed();

  if (!InitializeHardware()) {
    // Only a failure of the modulator itself gets here now. Without a working
    // duty output there is no safe state to be in, so latch the fault.
    tx_.EnterFault("modulator initialisation failed");
  }

  StartButtons();
  ui_task_.Start();

  ESP_LOGI(kTag, "AM Transmitter initialized");

  RunMainLoop();
}

void TransmitterSystem::RunMainLoop() {
  // The task watchdog covers this loop. If it stops feeding, the chip resets,
  // GPIO goes high impedance and the external pull-down takes the duty to 0
  // (spec 8.6).
  esp_task_wdt_add(nullptr);

  uint32_t last_tick_count = modulator_.GetTickCount();
  int64_t last_tick_check_ms = NowMs();

  while (true) {
    esp_task_wdt_reset();

    tx_.Tick();
    ardf_.Tick();

    // Battery supervision (spec 3.6). The inhibit decision runs on the one
    // second average, never the instantaneous reading, or every key down
    // would trip it as the pack sags under the 1.05A peak.
    if (adc_.IsReady()) {
      adc_.UpdateBattery();
      const float average = adc_.GetBatteryAverageVoltage();
      if (average > 0.0f) {
        tx_.SetInhibited(hardware_config::kBatInhibitEnabled &&
                         average < hardware_config::kBatInhibitV);
      }
      ui_task_.SetBatteryVoltage(adc_.GetBatteryVoltage());
    }

    // Watch the 25kHz tick itself. The Si5351A module has no OEB pin, so a
    // stalled ISR with the carrier up would transmit an unmodulated carrier
    // indefinitely; catching it here lets us shut down deliberately.
    const int64_t now = NowMs();
    if ((now - last_tick_check_ms) >=
        static_cast<int64_t>(hardware_config::kTickWatchIntervalMs)) {
      const uint32_t tick_count = modulator_.GetTickCount();
      if (tick_count == last_tick_count &&
          tx_.GetState() != TxState::kFault) {
        tx_.EnterFault("master tick stalled");
      }
      last_tick_count = tick_count;
      last_tick_check_ms = now;
    }

    UpdateStatusLed(now);

    util::SleepMillisecond(hardware_config::kMainLoopIntervalMs);
  }
}

}  // namespace transmitter_system

// EOF
