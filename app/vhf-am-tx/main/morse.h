#ifndef AM_TRANSMITTER_MAIN_MORSE_H_
#define AM_TRANSMITTER_MAIN_MORSE_H_
// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// Morse code table and ARDF identification signals (spec 10.1)

#include <cstdint>

namespace transmitter_system {
namespace morse {

/// Morse pattern for one character, packed into a byte pair.
///
/// `bits` holds the elements MSB first: 0 = dit, 1 = dah.
/// `length` says how many of them are valid.
struct Pattern {
  uint8_t bits;
  uint8_t length;
};

/// Looks up a character. Accepts 'A'-'Z' (upper or lower case) and '0'-'9',
/// which covers both callsigns and every ARDF identification signal - the
/// ARDF set (M O E I S H 5) is a subset, so one table serves both.
///
/// Returns a pattern with length 0 for anything else, which the keyer treats
/// as "nothing to send".
Pattern Lookup(char c);

/// ARDF identification signals (spec 10.1).
///
/// The choice of signal also picks the transmit slot, which is why they are
/// one enum and not two settings: the international rules pair them one to
/// one, and splitting them would only invite misconfiguration.
enum class ArdfId : uint8_t {
  kMoe = 0,  // -- --- .      slot 1, 0:00-1:00
  kMoi = 1,  // -- --- ..     slot 2, 1:00-2:00
  kMos = 2,  // -- --- ...    slot 3, 2:00-3:00
  kMoh = 3,  // -- --- ....   slot 4, 3:00-4:00
  kMo5 = 4,  // -- --- .....  slot 5, 4:00-5:00
  kMo = 5,   // -- ---        continuous (homing beacon / finish)
  kS = 6,    // ...          continuous (sprint spectator control)
  kCount = 7,
};

/// Text of an identification signal, e.g. "MOE". Never null.
const char* GetArdfIdText(ArdfId id);

/// Slot number 1-5, or 0 for kMo and kS which transmit continuously and have
/// no slot.
uint8_t GetArdfSlot(ArdfId id);

/// True for kMo (homing beacon) and kS (spectator): both transmit without
/// interruption.
inline bool IsContinuous(ArdfId id) {
  return id == ArdfId::kMo || id == ArdfId::kS;
}

/// Duration of a string in dit units, including inter-character gaps but not
/// any trailing word gap. Used to decide whether another repetition still
/// fits inside a slot (spec 10.4).
uint32_t CountUnits(const char* text);

// Element timing in dit units (spec 10.2)
constexpr uint32_t kDitUnits = 1;
constexpr uint32_t kDahUnits = 3;
constexpr uint32_t kIntraCharGapUnits = 1;
constexpr uint32_t kInterCharGapUnits = 3;
constexpr uint32_t kWordGapUnits = 7;

/// Unit length in milliseconds: 1200 / WPM.
inline uint32_t UnitMs(uint8_t wpm) { return wpm > 0 ? 1200u / wpm : 80u; }

}  // namespace morse
}  // namespace transmitter_system

#endif  // AM_TRANSMITTER_MAIN_MORSE_H_
