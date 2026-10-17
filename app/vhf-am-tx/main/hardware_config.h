#ifndef AM_TRANSMITTER_MAIN_HARDWARE_CONFIG_H_
#define AM_TRANSMITTER_MAIN_HARDWARE_CONFIG_H_
// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// Hardware Configuration Constants
//
// This is the single place where the constant table of spec chapter 11 lives.
// Constants marked CALIBRATE are expected to be replaced with measured values
// once the board is brought up; see the calibration procedure in spec 11.

#include <driver/gpio.h>
#include <sdkconfig.h>

#include <cstdint>

#include "xiao_esp32c6_pin.h"

namespace transmitter_system {
namespace hardware_config {

// ============================================================================
// GPIO Pin Assignments
//
// The schematic (design document chapter 04) is the authority for these.
// Firmware refers to buttons by role, never by the silkscreen number, so
// renumbering the switches does not reach into the code.
// ============================================================================

// PWM (modulation) output. GPIO2 has no boot-time function, so the external
// pull-down alone holds it at 0V during reset.
constexpr gpio_num_t kPwmPin = xiao_esp32c6_pin::kD2;  // GPIO2

// I2C bus (Si5351A + ST7032 LCD)
constexpr gpio_num_t kI2cSdaPin = xiao_esp32c6_pin::kSda;  // GPIO22 (D4)
constexpr gpio_num_t kI2cSclPin = xiao_esp32c6_pin::kScl;  // GPIO23 (D5)

// ADC inputs (ADC1 only; GPIO0-GPIO2 are the only usable pins)
constexpr gpio_num_t kBatteryVoltagePin = xiao_esp32c6_pin::kD0;  // GPIO0
constexpr gpio_num_t kMicrophonePin = xiao_esp32c6_pin::kD1;      // GPIO1

// Board LED (XIAO ESP32C6 user LED, lit by driving the pin low).
//   - solid while the carrier is up (TxController::IsTransmitting());
//   - otherwise blinks fast when the LCD is down, since then there is nowhere
//     else to show the problem;
//   - off otherwise.
constexpr gpio_num_t kStatusLedPin = xiao_esp32c6_pin::kLedBuiltin;  // GPIO15
constexpr bool kStatusLedActiveLow = true;
constexpr uint32_t kStatusLedBlinkMs = 50;  // toggle period -> 10 Hz

// Push buttons. Roles change per mode (spec 9.2); internal pull-ups required.
constexpr gpio_num_t kSwModePin = xiao_esp32c6_pin::kD10;     // GPIO18 (SW0)
constexpr gpio_num_t kSwFnPlusPin = xiao_esp32c6_pin::kD9;    // GPIO20 (SW1)
constexpr gpio_num_t kSwFnMinusPin = xiao_esp32c6_pin::kD8;   // GPIO19 (SW2)

// ============================================================================
// Master Timing (spec 7.1)
// ============================================================================

// Master tick. Everything on the modulation path updates at exactly this rate.
constexpr uint32_t kTickHz = 25000;
constexpr uint32_t kTickPeriodUs = 1000000 / kTickHz;  // 40us

// gptimer resolution: 1us steps
constexpr uint32_t kTickTimerResolutionHz = 1000000;

// ISR budget (spec 7.1). Measured at runtime and surfaced in the test modes.
constexpr uint32_t kIsrBudgetUs = 20;

// ============================================================================
// PWM / LEDC (spec 6.1)
// ============================================================================

// 80MHz / 250kHz = 320 -> 8 bit resolution (fractional divider 1.25).
// 9 bit does not fit: 80MHz / (250kHz * 512) = 0.625 < 1.
constexpr uint32_t kPwmHz = 250000;
constexpr uint32_t kPwmBits = 8;
constexpr uint32_t kPwmFullScale = 1u << kPwmBits;  // 256

// ---- Modulator circuit (design chapters 03/04). Everything below is derived
// from these; change R9 here and nothing else. ------------------------------
constexpr float kVddPwm = 3.3f;           // PWM high level [V]
constexpr float kModR8 = 10.0e3f;         // M3 feedback, FB -> GND
constexpr float kModR9 = 22.0e3f;         // M3 feedback, FB -> source
                                          // (20k -> 22k, ADR-024)
constexpr float kVDrainCarrier = 3.6f;    // carrier point (1.5W)
constexpr float kVDrainHardLimit = 7.75f; // margin under the 8V rail's 7.85V

// Drain voltage = M2 output * kModGain.
constexpr float kModGain = 1.0f + kModR9 / kModR8;  // 3.2

// Duty mapping (spec 2.2): D = kModK * (1 + x), nominal kModK = 0.34091.
constexpr float kModKNom = kVDrainCarrier / (kModGain * kVddPwm);

// CALIBRATE - measure the modulator at a fixed duty in D:DUTY and set
//   kModKCal = (duty / 256) * 3.6 / V_measured
// then rebuild. It is deliberately not stored in NVS. Anything outside +-5% of
// nominal means a wrong part or rail, not something to calibrate away.
constexpr float kModKCal = 0.3295f;  // final stage connected, C5 refitted: 84 -> 3.45V, 0.3158*3.6/3.45
constexpr float kModKCalMin = kModKNom * 0.95f;
constexpr float kModKCalMax = kModKNom * 1.05f;

// Duty at x = 0 (3.6V at the drain): 84 with the calibration above.
constexpr uint32_t kDutyCarrier =
    static_cast<uint32_t>(kModKCal * static_cast<float>(kPwmFullScale) + 0.5f);

// Absolute ceiling, applied in every mode including the test modes.
// floor(7.75 * 256 / (3.3 * kModGain)) = 187 -> 7.7V at the drain.
// Written out rather than derived from the calibration, so a wrong
// calibration cannot raise it. static_assert below ties it to the circuit.
// This is the last line of defence against a firmware bug. Do not change.
constexpr uint32_t kDutyHardMax = 187;

// Ceiling actually used by every path. The calibration can only lower it:
// a modulator with more gain than nominal reaches 7.75V at a smaller duty.
//   floor(7.75 / 3.6 * 256 * kModKCal) = 181 with kModKCal = 0.3295
constexpr uint32_t kDutyCalMax = static_cast<uint32_t>(
    kVDrainHardLimit / kVDrainCarrier * static_cast<float>(kPwmFullScale) *
    kModKCal);
constexpr uint32_t kDutyLimit =
    kDutyCalMax < kDutyHardMax ? kDutyCalMax : kDutyHardMax;
static_assert(kDutyLimit <= kDutyHardMax, "kDutyLimit above the absolute ceiling");

// Compile-time self-test. A mismatch here stops the build instead of putting
// too much on the drain.
static_assert(kModGain > 3.19f && kModGain < 3.21f, "MOD_GAIN must be 3.2");
static_assert(kDutyCarrier == 84, "carrier duty must be 84 for kModKCal 0.3295");
static_assert(kDutyCarrier < kDutyLimit, "carrier must sit below the ceiling");
static_assert(kDutyHardMax ==
                  static_cast<uint32_t>(kVDrainHardLimit *
                                        static_cast<float>(kPwmFullScale) /
                                        (kVddPwm * kModGain)),
              "kDutyHardMax does not match the circuit constants");
static_assert(static_cast<float>(kDutyHardMax) /
                      static_cast<float>(kPwmFullScale) * kVddPwm * kModGain <=
                  7.85f,
              "kDutyHardMax would exceed the 8V rail limit");
static_assert(kModKCal >= kModKCalMin && kModKCal <= kModKCalMax,
              "kModKCal is outside +-5% of nominal");

// ============================================================================
// Debug test modes (spec 12)
// ============================================================================

// Set by menuconfig: AM Transmitter -> Enable the debug test modes.
#ifdef CONFIG_AM_TX_DEBUG_MODES
constexpr bool kDebugModesEnabled = true;
#else
constexpr bool kDebugModesEnabled = false;
#endif

// D:BRST: a tone burst of kBurstOnMs at the start of every kBurstPeriodMs.
constexpr int64_t kBurstPeriodMs = 60000;
constexpr int64_t kBurstOnMs = 5000;

// D:CARR "P" level: carrier duty x 1.8 (84 -> 152), clamped by kDutyLimit.
// It heats the final stage, so it switches itself off after 3 seconds.
constexpr float kCarrierPeakFactor = 1.8f;
constexpr int64_t kCarrierPeakMaxMs = 3000;

// ============================================================================
// Modulation Input x (spec 2.1, 4.2, 5.1)
// ============================================================================

// A2A tone amplitude: 80% modulation / 0.877 (M2 loss at 700Hz).
// CALIBRATE together with the M2 component values.
constexpr float kXGainA2a = 0.912f;

// A3E asymmetric limiter (spec 4.2 / ADR-017).
// Negative side stops short of the knee voltage (0.6V); positive side uses
// the headroom left over on the 8V rail.
// CALIBRATE
constexpr float kXMinA3e = -0.85f;  // drain 0.54V
constexpr float kXMaxA3e = 1.10f;   // drain 7.56V

// Absolute clamp shared by every path
constexpr float kXAbsMin = -1.0f;
constexpr float kXAbsMax = 1.143f;  // maps to kDutyHardMax (187)

// ============================================================================
// Tone Generation (spec 5)
// ============================================================================

constexpr uint32_t kToneHz = 700;
constexpr uint32_t kSineTableBits = 8;
constexpr uint32_t kSineTableSize = 1u << kSineTableBits;  // 256

// Phase increment for a 32-bit accumulator: (700 / 25000) * 2^32
//   700 / 25000 = 0.028 exactly, 0.028 * 2^32 = 120259084.288 -> 0x072B020C
// which lands on 699.999998 Hz.
//
// NOTE: spec 5.1 prints 0x01CAC083 for this constant, which is exactly a
// quarter of the correct value and would sound a 175Hz tone. That figure
// matches a 2^30 accumulator, so it looks like a scaling slip in the spec.
// Using the corrected value here; spec 5.1 needs the same fix.
constexpr uint32_t kTonePhaseIncrement = 0x072B020Cu;

// Raised-cosine envelope, spec 5.2. 5ms at 25kHz = 125 samples.
constexpr float kEnvelopeMs = 5.0f;
constexpr uint32_t kEnvelopeSamples =
    static_cast<uint32_t>(kEnvelopeMs * kTickHz / 1000.0f);  // 125

// Voice chain design values (spec 1.2 / 4.1 / 4.4): M2 and the anti-alias
// filter share 1kohm x 47nF (RC = 47us), the EQ poles sit at 10kHz, and the
// band is limited to 300Hz-3kHz. The filter coefficients derived from these
// are hard-coded in voice_dsp.cc; change them there together.

// ============================================================================
// ADC (spec 3)
// ============================================================================

// Both channels run off ADC1 in continuous (DMA) mode. 50kS/s total means
// 25kS/s each, so the microphone lines up with the master tick.
constexpr uint32_t kAdcSampleFreqHz = 50000;
constexpr uint32_t kAdcPerChannelHz = kAdcSampleFreqHz / 2;  // 25kHz

// Clip detection thresholds on the raw 12-bit code (spec 3.3)
constexpr int kAdcClipLow = 50;
constexpr int kAdcClipHigh = 4045;
constexpr int kAdcClipRunLength = 3;

// DC bias tracking (spec 3.4). Startup average then a very slow 1-pole
// follower, fc = 1Hz: a = 2*pi*1/25000.
constexpr uint32_t kAdcBiasInitSamples = 5000;  // 200ms at 25kHz
constexpr float kAdcBiasAlpha = 2.51e-4f;

// Microphone ring buffer between the DMA callback and the 25kHz ISR
constexpr uint32_t kMicRingSize = 8;

// ============================================================================
// Battery Monitoring (spec 3.6)
// ============================================================================

// Divider: (100k + 18k) / 18k.
// CALIBRATE - the schematic does not carry these parts yet (spec 13-5).
// Measure the fitted resistors and match this value. Take the divider from
// downstream of the reverse-polarity protection.
constexpr float kBatDivRatio = 6.556f;

// 25kS/s raw -> 2500 sample average -> 10Hz
constexpr uint32_t kBatAvgSamples = 2500;

// Thresholds (spec 3.6)
constexpr float kBatWarnV = 11.5f;     // LCD warning, transmit still allowed
constexpr float kBatInhibitV = 10.5f;  // refuse to start, stop if running

// Set by menuconfig: AM Transmitter -> Inhibit transmission on low battery.
// Turn it off for bench work on USB power; the LCD "LOW BATT" pin goes with it.
#ifdef CONFIG_AM_TX_BATTERY_INHIBIT
constexpr bool kBatInhibitEnabled = true;
#else
constexpr bool kBatInhibitEnabled = false;
#endif

// The inhibit decision runs on a 1 second moving average of the 10Hz value.
// Judging on the instantaneous reading would cut transmission on every key
// down, because peaks draw 1.05A and the pack sags 0.3-0.5V.
constexpr uint32_t kBatInhibitAvgCount = 10;  // 10 x 100ms = 1s

// ============================================================================
// Frequency Control (spec 9.4.3)
// ============================================================================

constexpr uint32_t kFreqStepHz = 20000;

// CALIBRATE - set these to the band segment actually in use. Narrowing the
// range cuts the number of button presses; the operator is responsible for
// checking the band plan.
constexpr uint32_t kFreqMinHz = 144000000;
constexpr uint32_t kFreqMaxHz = 146000000;
constexpr uint32_t kFreqDefaultHz = 145700000;

// Write to the Si5351A this long after the button is released. Writing on
// every press would choke the I2C bus.
constexpr uint32_t kFreqWriteDelayMs = 300;

// ============================================================================
// CW / ARDF (spec 10)
// ============================================================================

// Unit length = 1200 / WPM milliseconds. ARDF typically runs 8-15 WPM.
// CALIBRATE (operator preference)
constexpr uint8_t kCwWpmDefault = 8;
constexpr uint8_t kCwWpmMin = 5;
constexpr uint8_t kCwWpmMax = 40;

constexpr uint32_t kArdfCycleS = 300;  // 5 minute cycle
constexpr uint32_t kArdfSlotS = 60;    // 1 minute slot
constexpr uint8_t kArdfIdDefault = 0;  // MOE

// MO continuous mode inserts the callsign every 10 minutes
constexpr uint32_t kMoCallIntervalS = 600;

// Callsign length (spec 9.4.4). There is no built-in default: until one is
// entered in CALL mode and saved to NVS the callsign is unset, the title
// screen leaves it blank and ARDF refuses to start (spec 10.5 requires it).
constexpr uint8_t kCallLength = 6;

// ============================================================================
// Buttons (spec 9.7)
// ============================================================================

constexpr uint32_t kButtonPollIntervalMs = 5;
constexpr uint32_t kBtnDebounceMs = 20;

// Auto-repeat applies to FN+/FN- only. SW_MODE is always single shot, and so
// are the voice PTT and the ARDF run toggle.
constexpr uint32_t kBtnRepeatDelayMs = 600;
constexpr uint32_t kBtnRepeatSlowHz = 8;
constexpr uint32_t kBtnRepeatFastAfterMs = 2000;
constexpr uint32_t kBtnRepeatFastHz = 25;

constexpr uint32_t kButtonEventQueueSize = 16;

// ============================================================================
// Transmit Sequencing (spec 8.2 / 8.3)
// ============================================================================

// Duty ramps 0 <-> carrier over 5ms with a raised cosine.
constexpr uint32_t kTxRampMs = 5;
constexpr uint32_t kTxRampSamples = kTxRampMs * kTickHz / 1000;  // 125

// After ramping down, wait for the drain to fall before killing CLK0.
constexpr uint32_t kTxStopSettleMs = 10;

// PLL lock wait after enabling CLK0
constexpr uint32_t kPllLockWaitMs = 10;

// ============================================================================
// Watchdog (spec 8.6)
// ============================================================================

// The Si5351A module has no OEB pin, so I2C is the only way to stop the
// carrier: a hung MCU would otherwise transmit forever. The main loop checks
// that the master tick is still advancing at this interval. The task
// watchdog timeout itself (3s) is CONFIG_ESP_TASK_WDT_TIMEOUT_S in
// sdkconfig.defaults.
constexpr uint32_t kTickWatchIntervalMs = 100;

// ============================================================================
// NVS (spec 9.6)
// ============================================================================

// Settings are written this long after the last change, and on mode switch.
// Writing on every button press would burn flash endurance for nothing.
constexpr uint32_t kNvsWriteDelayMs = 3000;

// ============================================================================
// I2C
// ============================================================================

// 100kHz: the bus may rely on weak internal pull-ups (no external ones on the
// AQM0802A/Si5351A modules), and 400kHz edges are too slow to read reliably.
constexpr uint32_t kI2cFrequencyHz = 100000;
constexpr uint32_t kI2cBusStabilizeDelayMs = 100;

// ============================================================================
// Display (spec 9.5) - ST7032, 16 characters x 2 lines
// ============================================================================

constexpr uint8_t kLcdColumns = 16;
constexpr uint8_t kLcdLines = 2;
constexpr uint8_t kLcdContrast = 40;
constexpr uint32_t kDisplayUpdateIntervalMs = 100;

// Title screen after power-on: kBootTitle on the top line, the callsign on the
// second. Start-up problems, if any, follow it one by one.
constexpr const char* kBootTitle = "AM Transmitter";
constexpr uint32_t kBootTitleMs = 2000;

// ============================================================================
// Main loop (TX sequencing, ARDF schedule, battery and tick supervision)
// ============================================================================

constexpr uint32_t kMainLoopIntervalMs = 10;

}  // namespace hardware_config
}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_HARDWARE_CONFIG_H_

// EOF
