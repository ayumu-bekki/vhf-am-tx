#ifndef AM_TRANSMITTER_MAIN_CW_KEYER_H_
#define AM_TRANSMITTER_MAIN_CW_KEYER_H_
// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// CW keyer (spec 10.3)

#include <esp_attr.h>

#include <atomic>
#include <cstdint>

#include "hardware_config.h"
#include "morse.h"
#include "tone_generator.h"

namespace transmitter_system {

/// Keys a Morse string, driven one step per 25kHz tick.
///
/// The keyer only decides key up / key down; the tone generator turns that
/// into x, including the 5ms raised-cosine edges that keep the occupied
/// bandwidth in check (spec 10.3).
///
/// The carrier is never keyed here. A2A keys the tone and leaves the carrier
/// running, which is the whole point of the mode.
class CwKeyer {
 public:
  /// Longest string the keyer will accept. A callsign is 6 characters and the
  /// ARDF identifiers are 3, so this is generous.
  static constexpr size_t kMaxTextLength = 32;

  CwKeyer();

  void SetToneGenerator(ToneGenerator* tone) { tone_ = tone; }

  /// Words per minute. Takes effect on the next element.
  void SetWpm(uint8_t wpm);
  uint8_t GetWpm() const { return wpm_; }

  /// Loads a string and starts sending it. Any string in progress is dropped.
  /// Safe to call from a task while the ISR is running: the text is staged and
  /// picked up on the next tick.
  void Send(const char* text);

  /// Stops at the next tick and releases the key (the envelope still decays
  /// over 5ms). Cancels a Send() that has not started yet.
  void Stop();

  /// True while there is still something to send (including trailing gaps).
  bool IsBusy() const { return busy_.load(std::memory_order_acquire); }

  /// Advances one 25kHz tick. Returns the modulation input x for this sample.
  float IRAM_ATTR Tick();

 private:
  enum class Phase : uint8_t {
    kIdle,
    kElement,      // key down for a dit or dah
    kIntraGap,     // gap between elements of one character
    kInterGap,     // gap between characters
    kWordGap,      // gap for an explicit space
    kTailGap,      // let the envelope decay after the last element
  };

  void IRAM_ATTR ResetSequence();
  void IRAM_ATTR StartNextElement();
  void IRAM_ATTR AdvanceCharacter();
  uint32_t IRAM_ATTR SamplesForUnits(uint32_t units) const;

 private:
  ToneGenerator* tone_;

  // Text being sent, plus the staging buffer written by the task side.
  char text_[kMaxTextLength + 1];
  char pending_text_[kMaxTextLength + 1];
  std::atomic<bool> pending_load_;
  std::atomic<bool> busy_;
  std::atomic<bool> stop_request_;

  size_t char_index_;
  morse::Pattern pattern_;
  uint8_t element_index_;

  Phase phase_;
  uint32_t phase_samples_remaining_;

  uint8_t wpm_;
  uint32_t samples_per_unit_;
};

}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_CW_KEYER_H_
