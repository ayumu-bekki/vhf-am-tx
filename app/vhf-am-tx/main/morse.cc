// ESP32 144MHz AM Transmitter
// (C)2025 bekki.jp
// Morse code table and ARDF identification signals (spec 10.1)

#include "morse.h"

namespace transmitter_system {
namespace morse {

namespace {

// Bits are read MSB first within `length`: 0 = dit, 1 = dah.
// 0b10000000 with length 1 is a single dah.
constexpr Pattern kLetters[26] = {
    {0b01000000, 2},  // A .-
    {0b10000000, 4},  // B -...
    {0b10100000, 4},  // C -.-.
    {0b10000000, 3},  // D -..
    {0b00000000, 1},  // E .
    {0b00100000, 4},  // F ..-.
    {0b11000000, 3},  // G --.
    {0b00000000, 4},  // H ....
    {0b00000000, 2},  // I ..
    {0b01110000, 4},  // J .---
    {0b10100000, 3},  // K -.-
    {0b01000000, 4},  // L .-..
    {0b11000000, 2},  // M --
    {0b10000000, 2},  // N -.
    {0b11100000, 3},  // O ---
    {0b01100000, 4},  // P .--.
    {0b11010000, 4},  // Q --.-
    {0b01000000, 3},  // R .-.
    {0b00000000, 3},  // S ...
    {0b10000000, 1},  // T -
    {0b00100000, 3},  // U ..-
    {0b00010000, 4},  // V ...-
    {0b01100000, 3},  // W .--
    {0b10010000, 4},  // X -..-
    {0b10110000, 4},  // Y -.--
    {0b11000000, 4},  // Z --..
};

constexpr Pattern kDigits[10] = {
    {0b11111000, 5},  // 0 -----
    {0b01111000, 5},  // 1 .----
    {0b00111000, 5},  // 2 ..---
    {0b00011000, 5},  // 3 ...--
    {0b00001000, 5},  // 4 ....-
    {0b00000000, 5},  // 5 .....
    {0b10000000, 5},  // 6 -....
    {0b11000000, 5},  // 7 --...
    {0b11100000, 5},  // 8 ---..
    {0b11110000, 5},  // 9 ----.
};

struct ArdfEntry {
  const char* text;
  uint8_t slot;
};

constexpr ArdfEntry kArdfTable[static_cast<uint8_t>(ArdfId::kCount)] = {
    {"MOE", 1}, {"MOI", 2}, {"MOS", 3}, {"MOH", 4}, {"MO5", 5}, {"MO", 0},
    {"S", 0},
};

}  // namespace

Pattern Lookup(char c) {
  if (c >= 'a' && c <= 'z') {
    c = static_cast<char>(c - 'a' + 'A');
  }
  if (c >= 'A' && c <= 'Z') {
    return kLetters[c - 'A'];
  }
  if (c >= '0' && c <= '9') {
    return kDigits[c - '0'];
  }
  return {0, 0};
}

const char* GetArdfIdText(ArdfId id) {
  const uint8_t index = static_cast<uint8_t>(id);
  if (index >= static_cast<uint8_t>(ArdfId::kCount)) {
    return kArdfTable[0].text;
  }
  return kArdfTable[index].text;
}

uint8_t GetArdfSlot(ArdfId id) {
  const uint8_t index = static_cast<uint8_t>(id);
  if (index >= static_cast<uint8_t>(ArdfId::kCount)) {
    return kArdfTable[0].slot;
  }
  return kArdfTable[index].slot;
}

uint32_t CountUnits(const char* text) {
  if (!text) {
    return 0;
  }

  uint32_t units = 0;
  bool first_char = true;

  for (const char* p = text; *p != '\0'; ++p) {
    if (*p == ' ') {
      // A space asks for a word gap. The inter-character gap that would
      // otherwise follow the previous character is subsumed by it.
      if (!first_char) {
        units += kWordGapUnits - kInterCharGapUnits;
      }
      continue;
    }

    const Pattern pattern = Lookup(*p);
    if (pattern.length == 0) {
      continue;
    }

    if (!first_char) {
      units += kInterCharGapUnits;
    }
    first_char = false;

    for (uint8_t i = 0; i < pattern.length; ++i) {
      const bool is_dah = (pattern.bits >> (7 - i)) & 0x01;
      units += is_dah ? kDahUnits : kDitUnits;
      if (i + 1 < pattern.length) {
        units += kIntraCharGapUnits;
      }
    }
  }

  return units;
}

}  // namespace morse
}  // namespace transmitter_system

// EOF
