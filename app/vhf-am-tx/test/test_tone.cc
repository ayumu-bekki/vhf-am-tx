// Host test: the tone pitch is selectable in 1Hz steps, 440..2093Hz.
#include <cmath>
#include <cstdio>
#include <initializer_list>

#include "tone_generator.h"

using namespace transmitter_system;

int main() {
  int failures = 0;
  auto check = [&](bool ok, const char* what) {
    printf("  %-62s %s\n", what, ok ? "OK" : "MISMATCH");
    if (!ok) ++failures;
  };

  printf("--- default ---\n");
  {
    ToneGenerator t;
    check(t.GetFrequencyHz() == 698, "default is 698Hz");
  }

  printf("\n--- generated frequency (rising zero crossings in 1 s) ---\n");
  for (int hz : {440, 600, 698, 700, 800, 900, 1000, 1234, 2093}) {
    ToneGenerator t;
    t.Initialize();
    t.SetFrequencyHz(static_cast<uint16_t>(hz));
    t.SetKey(true);
    for (int i = 0; i < 1000; ++i) t.Next();  // let the envelope open
    int crossings = 0;
    float prev = t.Next();
    for (uint32_t i = 0; i < hardware_config::kTickHz; ++i) {
      const float v = t.Next();
      if (prev < 0.0f && v >= 0.0f) ++crossings;
      prev = v;
    }
    char what[64];
    std::snprintf(what, sizeof(what), "%dHz: %d crossings", hz, crossings);
    check(std::abs(crossings - hz) <= 1, what);
  }

  printf("\n--- range ---\n");
  {
    ToneGenerator t;
    t.SetFrequencyHz(100);
    check(t.GetFrequencyHz() == 440, "below range is clamped to 440Hz");
    t.SetFrequencyHz(5000);
    check(t.GetFrequencyHz() == 2093, "above range is clamped to 2093Hz");
  }

  printf("\n%s\n", failures ? "FAILED" : "tone tests passed");
  return failures ? 1 : 0;
}
