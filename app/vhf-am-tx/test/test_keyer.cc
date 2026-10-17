// Host test: CW keyer + tone generator, driven one 25kHz tick at a time.
// Checks the element/gap timing the ISR would produce against spec 10.2.
#include <cstdio>
#include <cstring>
#include <vector>
#include <string>
#include <cmath>

#include "tone_generator.h"
#include "cw_keyer.h"
#include "morse.h"

using namespace transmitter_system;

int64_t g_fake_time_us = 0;

// Decode the key-down/key-up pattern the keyer produces back into dits/dahs,
// so we verify observable behaviour rather than internal state.
struct Segment { bool key; uint32_t samples; };

static std::vector<Segment> RunKeyer(CwKeyer& k, ToneGenerator& t,
                                     const char* text, uint32_t max_ticks) {
    k.Send(text);
    std::vector<Segment> segs;
    bool prev = false; uint32_t run = 0; uint32_t idle_after = 0;
    for (uint32_t i = 0; i < max_ticks; ++i) {
        k.Tick();
        const bool key = t.IsKeyDown();
        if (i == 0) { prev = key; run = 1; continue; }
        if (key == prev) { ++run; }
        else { segs.push_back({prev, run}); prev = key; run = 1; }
        if (!k.IsBusy()) { if (++idle_after > 200) break; }
    }
    segs.push_back({prev, run});
    return segs;
}

int main() {
    const uint32_t WPM = 15;
    const uint32_t unit = 1200u * 25000u / (WPM * 1000u);  // samples per dit
    printf("WPM=%u -> 1 unit = %u samples (%.1f ms)\n\n", WPM, unit, unit/25.0);

    int failures = 0;

    // --- 1. Element timing for a known string -----------------------------
    {
        ToneGenerator tone; tone.Initialize();
        CwKeyer k; k.SetToneGenerator(&tone); k.SetWpm(WPM);
        auto segs = RunKeyer(k, tone, "MOE", 25000 * 5);

        printf("--- MOE key pattern ---\n");
        // M=--, O=---, E=.  expected key-down runs: 3,3, 3,3,3, 1 (units)
        std::vector<uint32_t> want_down = {3,3, 3,3,3, 1};
        std::vector<uint32_t> got_down;
        for (auto& s : segs) if (s.key) got_down.push_back((s.samples + unit/2) / unit);

        printf("  key-down (units): ");
        for (auto u : got_down) printf("%u ", u);
        printf("\n  expected       : ");
        for (auto u : want_down) printf("%u ", u);
        printf("\n");
        if (got_down != want_down) { printf("  MISMATCH\n"); ++failures; }
        else printf("  OK\n");

        // Gaps between elements: 1 unit inside a character, 3 between
        std::vector<uint32_t> got_gap;
        for (size_t i = 1; i + 1 < segs.size(); ++i)
            if (!segs[i].key) got_gap.push_back((segs[i].samples + unit/2) / unit);
        std::vector<uint32_t> want_gap = {1, 3, 1, 1, 3};  // M|,|O..|,|E
        printf("  gaps (units)   : ");
        for (auto u : got_gap) printf("%u ", u);
        printf("\n  expected       : ");
        for (auto u : want_gap) printf("%u ", u);
        printf("\n");
        if (got_gap != want_gap) { printf("  MISMATCH\n"); ++failures; }
        else printf("  OK\n");
    }

    // --- 2. Total duration matches spec 10.2 ------------------------------
    printf("\n--- total duration vs spec 10.2 ---\n");
    struct { const char* t; double sec; } cases[] = {
        {"MO",1.68},{"MOE",2.00},{"MOI",2.16},{"MOS",2.32},
        {"MOH",2.48},{"MO5",2.64},{"JL1HMT",5.68},
    };
    for (auto& c : cases) {
        ToneGenerator tone; tone.Initialize();
        CwKeyer k; k.SetToneGenerator(&tone); k.SetWpm(WPM);
        k.Send(c.t);
        uint32_t n = 0, last_down = 0;
        for (uint32_t i = 0; i < 25000*10; ++i) {
            k.Tick(); ++n;
            if (tone.IsKeyDown()) last_down = n;
            if (!k.IsBusy() && i > 10) break;
        }
        const double sec = last_down / 25000.0;
        const bool ok = std::fabs(sec - c.sec) < 0.02;
        printf("  %-7s %.2fs (spec %.2fs) %s\n", c.t, sec, c.sec, ok?"OK":"MISMATCH");
        if (!ok) ++failures;
    }

    // --- 3. Word gap handling ---------------------------------------------
    printf("\n--- word gap ---\n");
    {
        ToneGenerator tone; tone.Initialize();
        CwKeyer k; k.SetToneGenerator(&tone); k.SetWpm(WPM);
        auto segs = RunKeyer(k, tone, "E E", 25000*5);
        uint32_t gap = 0;
        for (size_t i = 1; i + 1 < segs.size(); ++i) if (!segs[i].key) gap = segs[i].samples;
        const uint32_t units = (gap + unit/2)/unit;
        printf("  'E E' inter-word gap = %u units (expect 7)  %s\n",
               units, units==7?"OK":"MISMATCH");
        if (units != 7) ++failures;
    }

    // --- 4. Envelope is applied (no hard edges) ---------------------------
    printf("\n--- keying envelope (spec 5.2) ---\n");
    {
        ToneGenerator tone; tone.Initialize();
        tone.SetKey(true);
        float peak_first = 0.0f;
        for (uint32_t i = 0; i < 5; ++i) peak_first = std::fmax(peak_first, std::fabs(tone.Next()));
        for (uint32_t i = 0; i < 25000/100; ++i) tone.Next();  // settle
        float peak_steady = 0.0f;
        for (uint32_t i = 0; i < 200; ++i) peak_steady = std::fmax(peak_steady, std::fabs(tone.Next()));
        printf("  first 5 samples peak = %.4f, steady peak = %.4f\n", peak_first, peak_steady);
        const bool ramped = peak_first < peak_steady * 0.25f;
        printf("  ramps in smoothly: %s\n", ramped ? "OK" : "MISMATCH (hard edge!)");
        if (!ramped) ++failures;
        const bool amp_ok = std::fabs(peak_steady - 0.912f) < 0.02f;
        printf("  steady amplitude = %.4f (spec 0.912) %s\n", peak_steady, amp_ok?"OK":"MISMATCH");
        if (!amp_ok) ++failures;
    }

    printf("\n===== %s (%d failure%s) =====\n",
           failures ? "FAILURES" : "ALL PASS", failures, failures==1?"":"s");
    return failures != 0;
}
