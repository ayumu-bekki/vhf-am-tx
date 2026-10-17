#ifndef AM_TRANSMITTER_MAIN_ARDF_SCHEDULER_H_
#define AM_TRANSMITTER_MAIN_ARDF_SCHEDULER_H_
// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// ARDF transmit schedule (spec 10.4)

#include <cstdint>

#include "cw_keyer.h"
#include "morse.h"
#include "tx_controller.h"

namespace transmitter_system {

/// Runs the five minute ARDF cycle.
///
/// Timing is taken from esp_timer's 64-bit microsecond clock and accumulated
/// as absolute deadlines, never by adding up vTaskDelay intervals, so the
/// error does not creep over a multi-hour session (spec 10.6). A +-20ppm
/// crystal drifts about 144ms over two hours, which is far inside what the
/// sport needs.
class ArdfScheduler {
 public:
  ArdfScheduler();

  void SetTxController(TxController* tx) { tx_ = tx; }
  void SetCwKeyer(CwKeyer* keyer) { keyer_ = keyer; }

  void SetArdfId(morse::ArdfId id) { ardf_id_ = id; }
  morse::ArdfId GetArdfId() const { return ardf_id_; }

  void SetCallsign(const char* callsign);

  /// Forwards to the keyer. Takes effect from the next string sent.
  void SetWpm(uint8_t wpm) {
    if (keyer_) {
      keyer_->SetWpm(wpm);
    }
  }

  /// Starts a run. The instant this is called is t = 0 of the five minute
  /// cycle (spec 10.6); operators either start their machines together or
  /// press at their own slot boundary.
  void Start();

  /// Ends the run, stopping any transmission in progress.
  void Stop();

  bool IsRunning() const { return running_; }

  /// Seconds until this station's next transmission starts. 0 while it is
  /// transmitting. Used for the countdown on the display.
  uint32_t GetSecondsToNextSlot() const;

  /// Advances the schedule. Call every few tens of milliseconds.
  void Tick();

 private:
  enum class Phase : uint8_t {
    kWaiting,      // between slots, carrier down
    kStarting,     // start sequence running
    kSendingCall,  // callsign going out
    kSendingId,    // identification signal repeating
    kStopping,     // stop sequence running
  };

  /// Microseconds into the current cycle.
  int64_t CycleElapsedUs() const;

  /// True when the clock is inside this station's slot.
  bool IsInOwnSlot() const;

  /// Whether another repetition of the identification signal still fits
  /// before the slot ends (spec 10.4): a partial send would be worse than
  /// silence, so it is cut at the last complete one.
  bool IdRepetitionFits() const;

  uint32_t UnitsToMs(uint32_t units) const;

 private:
  TxController* tx_;
  CwKeyer* keyer_;

  morse::ArdfId ardf_id_;
  char callsign_[16];

  bool running_;
  Phase phase_;

  /// Set when this slot's transmission has ended. The slot may still have
  /// seconds left, but a restart would overrun it into the next station's
  /// slot, so nothing starts again until the clock leaves the slot.
  bool slot_done_;

  /// esp_timer value at t = 0 of the cycle.
  int64_t cycle_start_us_;

  /// When the callsign was last sent, for the MO continuous mode which
  /// re-identifies every 10 minutes rather than every slot (spec 10.5).
  int64_t last_call_us_;
};

}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_ARDF_SCHEDULER_H_
