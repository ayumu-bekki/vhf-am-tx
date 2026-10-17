// Host test: ARDF scheduler over a full 5 minute cycle, with time simulated.
// This is the part that is painful to check on hardware - a single cycle takes
// five minutes and the interesting moments are the slot boundaries.
#include <cstdio>
#include <cmath>
#include <vector>
#include <string>

#include "ardf_scheduler.h"
#include "tx_controller.h"
#include "cw_keyer.h"
#include "tone_generator.h"
#include "morse.h"
#include "modulator.h"

using namespace transmitter_system;
int64_t g_fake_time_us = 0;

int main() {
    int failures = 0;

    // --- 1. Slot windows ---------------------------------------------------
    printf("--- slot windows (spec 10.1) ---\n");
    struct { morse::ArdfId id; const char* name; uint8_t slot; } ids[] = {
        {morse::ArdfId::kMoe,"MOE",1},{morse::ArdfId::kMoi,"MOI",2},
        {morse::ArdfId::kMos,"MOS",3},{morse::ArdfId::kMoh,"MOH",4},
        {morse::ArdfId::kMo5,"MO5",5},{morse::ArdfId::kMo ,"MO" ,0},
        {morse::ArdfId::kS  ,"S"  ,0},
    };
    for (auto& e : ids) {
        const char* t = morse::GetArdfIdText(e.id);
        const uint8_t s = morse::GetArdfSlot(e.id);
        const bool ok = (std::string(t)==e.name) && (s==e.slot);
        printf("  %-4s slot %u  %s\n", t, s, ok?"OK":"MISMATCH");
        if (!ok) ++failures;
    }

    // --- 2. Countdown across a cycle --------------------------------------
    printf("\n--- countdown to own slot (MOS = slot 3, 2:00-3:00) ---\n");
    {
        ArdfScheduler sch;
        CwKeyer k; ToneGenerator tone; tone.Initialize();
        k.SetToneGenerator(&tone);
        sch.SetCwKeyer(&k);
        sch.SetArdfId(morse::ArdfId::kMos);
        g_fake_time_us = 0;
        sch.Start();

        struct { int at_s; uint32_t want; } probes[] = {
            {0,120},{30,90},{60,60},{119,1},{120,0},{150,0},{179,0},{180,300-180+120},
        };
        for (auto& p : probes) {
            g_fake_time_us = (int64_t)p.at_s * 1000000;
            const uint32_t got = sch.GetSecondsToNextSlot();
            // inside the slot the countdown is 0; after it, wait for next cycle
            uint32_t want = p.want;
            if (p.at_s >= 120 && p.at_s < 180) want = 0;
            if (p.at_s >= 180) want = 300 - p.at_s + 120;
            const bool ok = got == want;
            printf("  t=%3ds -> %3us (expect %3us) %s\n", p.at_s, got, want, ok?"OK":"MISMATCH");
            if (!ok) ++failures;
        }
    }

    // --- 3. MO is continuous ----------------------------------------------
    printf("\n--- MO continuous ---\n");
    {
        ArdfScheduler sch;
        CwKeyer k; ToneGenerator tone; tone.Initialize();
        k.SetToneGenerator(&tone); sch.SetCwKeyer(&k);
        sch.SetArdfId(morse::ArdfId::kMo);
        g_fake_time_us = 0; sch.Start();
        bool all_zero = true;
        for (int t = 0; t < 300; t += 17) {
            g_fake_time_us = (int64_t)t * 1000000;
            if (sch.GetSecondsToNextSlot() != 0) all_zero = false;
        }
        printf("  countdown always 0 across the cycle: %s\n", all_zero?"OK":"MISMATCH");
        if (!all_zero) ++failures;
    }

    // --- 4. Full run through a cycle, observing the state machine ---------
    printf("\n--- full cycle walk (MOE = slot 1, 0:00-1:00) ---\n");
    {
        Modulator mod;           // not initialised; TxController tolerates it
        ArdfScheduler sch;
        TxController tx;
        CwKeyer k; ToneGenerator tone; tone.Initialize();
        k.SetToneGenerator(&tone); k.SetWpm(15);
        sch.SetCwKeyer(&k);
        sch.SetTxController(&tx);
        sch.SetArdfId(morse::ArdfId::kMoe);
        sch.SetCallsign("JL1HMT");

        g_fake_time_us = 0;
        sch.Start();
        printf("  ardf_running after Start(): %s\n", tx.IsArdfRunning()?"true OK":"false MISMATCH");
        if (!tx.IsArdfRunning()) ++failures;

        // Walk the cycle in 1s steps; the scheduler needs TxController to be
        // driven too, but without a modulator it stays in IDLE - so we only
        // assert on what is observable without hardware.
        int in_slot = 0, out_slot = 0;
        for (int t = 0; t < 300; ++t) {
            g_fake_time_us = (int64_t)t * 1000000;
            sch.Tick();
            if (sch.GetSecondsToNextSlot() == 0) ++in_slot; else ++out_slot;
        }
        printf("  seconds reported 'in slot' = %d (expect ~60)\n", in_slot);
        const bool ok = in_slot >= 59 && in_slot <= 61;
        printf("  slot width correct: %s\n", ok?"OK":"MISMATCH");
        if (!ok) ++failures;

        sch.Stop();
        printf("  ardf_running after Stop(): %s\n", !tx.IsArdfRunning()?"false OK":"true MISMATCH");
        if (tx.IsArdfRunning()) ++failures;
    }

    // --- 5. Repetition count fits the slot (spec 10.4) --------------------
    printf("\n--- ID repetitions per slot (15 WPM, JL1HMT) ---\n");
    {
        struct { const char* id; int want; } exp[] = {
            {"MOE",20},{"MOI",19},{"MOS",18},{"MOH",17},{"MO5",16},
        };
        const uint32_t unit_ms = 1200/15;
        const uint32_t call_ms = morse::CountUnits("JL1HMT")*unit_ms
                               + morse::kWordGapUnits*unit_ms;
        for (auto& e : exp) {
            const uint32_t rep_ms =
                (morse::CountUnits(e.id) + morse::kWordGapUnits) * unit_ms;
            uint32_t t = call_ms; int n = 0;
            while ((60000 - t) > rep_ms + 50) { t += rep_ms; ++n; }
            const bool ok = n == e.want;
            printf("  %-4s %2d reps (spec %2d) %s\n", e.id, n, e.want, ok?"OK":"MISMATCH");
            if (!ok) ++failures;
        }
    }

    printf("\n===== %s (%d failure%s) =====\n",
           failures?"FAILURES":"ALL PASS", failures, failures==1?"":"s");
    return failures != 0;
}
