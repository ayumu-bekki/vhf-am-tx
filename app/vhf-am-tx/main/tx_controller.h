#ifndef AM_TRANSMITTER_MAIN_TX_CONTROLLER_H_
#define AM_TRANSMITTER_MAIN_TX_CONTROLLER_H_
// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// Transmit sequencing and state machine (spec 8)

#include <atomic>
#include <cstdint>

#include "modulator.h"
#include "si5351a.h"

namespace transmitter_system {

/// Transmit states (spec 8.1).
enum class TxState : uint8_t {
  kIdle,        // CLK0 stopped, duty 0
  kTxStarting,  // CLK0 on, waiting for PLL lock, then ramping duty up
  kTxReady,     // carrier only (x = 0), waiting to be keyed
  kTxKeyed,     // modulating
  kTxStopping,  // ramping duty down, settling, then CLK0 off
  kFault,       // something went wrong; only a restart clears this
};

/// Drives the carrier and the modulation envelope in the right order.
///
/// The ordering in the stop sequence is not negotiable (spec 8.3): killing the
/// carrier before the drive has ramped down dumps 1.5W instantaneously and
/// splatters the sidebands across tens of kHz, and because the source follower
/// cannot sink current the modulation node would be left floating at 3.6V.
/// Everything here exists to make that ordering impossible to get wrong.
class TxController {
 public:
  TxController();

  void SetModulator(Modulator* modulator) { modulator_ = modulator; }
  void SetSi5351A(Si5351A* si5351a) { si5351a_ = si5351a; }

  TxState GetState() const { return state_.load(std::memory_order_acquire); }

  /// True while the carrier is up in any form.
  bool IsTransmitting() const {
    const TxState s = GetState();
    return s == TxState::kTxStarting || s == TxState::kTxReady ||
           s == TxState::kTxKeyed || s == TxState::kTxStopping;
  }

  /// ARDF "running" flag, deliberately orthogonal to the state (spec 8.1).
  ///
  /// An ARDF run spends four of every five minutes in kIdle, so the state
  /// alone cannot answer "is a run in progress". Mode switching is barred on
  /// this flag rather than on transmission, or the operator could walk past
  /// the mode during a silent slot and knock the station off the air mid
  /// competition (spec 9.3).
  bool IsArdfRunning() const {
    return ardf_running_.load(std::memory_order_acquire);
  }
  void SetArdfRunning(bool running) {
    ardf_running_.store(running, std::memory_order_release);
  }

  /// Requests the start sequence (spec 8.2). Returns false if transmission is
  /// inhibited (low battery) or the controller is faulted. While the carrier
  /// is already up this is a no-op; during kTxStopping the start is queued
  /// and runs once the stop has completed.
  bool RequestStart();

  /// Requests the stop sequence (spec 8.3). Also cancels a queued start.
  void RequestStop();

  /// Key down / key up once in kTxReady or kTxKeyed.
  void SetKeyed(bool keyed);

  /// Latches the fault state: duty to 0, carrier off, and stay there.
  void EnterFault(const char* reason);

  /// Blocks new transmissions and stops any in progress. Driven by the
  /// battery monitor (spec 3.6).
  void SetInhibited(bool inhibited);
  bool IsInhibited() const {
    return inhibited_.load(std::memory_order_acquire);
  }

  /// Advances the sequence. Call every few milliseconds from a task; the
  /// timing-critical part (the duty ramp) runs in the ISR, this only walks
  /// the state machine between phases.
  void Tick();

  /// Which source the modulator should read while keyed. Set by the UI when
  /// the mode changes.
  void SetKeyedSource(ModulationSource source) {
    keyed_source_.store(source, std::memory_order_release);
  }

 private:
  void TransitionTo(TxState state);

 private:
  Modulator* modulator_;
  Si5351A* si5351a_;

  std::atomic<TxState> state_;
  std::atomic<bool> ardf_running_;
  std::atomic<bool> inhibited_;
  std::atomic<bool> start_request_;
  std::atomic<bool> stop_request_;
  std::atomic<bool> keyed_request_;

  std::atomic<ModulationSource> keyed_source_;

  /// Timestamp the current phase started, for the settle delays.
  int64_t phase_start_ms_;
};

}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_TX_CONTROLLER_H_
