// Host test: edge cases and misuse, looking for real defects rather than
// confirming the happy path.
#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include "morse.h"
#include "tone_generator.h"
#include "cw_keyer.h"
#include "ardf_scheduler.h"
#include "tx_controller.h"

using namespace transmitter_system;
int64_t g_fake_time_us = 0;

int fails = 0;
static void Check(bool ok, const char* what) {
    printf("  %-58s %s\n", what, ok ? "OK" : "*** FAIL ***");
    if (!ok) ++fails;
}

int main() {
    printf("--- morse: unsupported input ---\n");
    Check(morse::Lookup('?').length == 0, "'?' returns length 0");
    Check(morse::Lookup(' ').length == 0, "space returns length 0");
    Check(morse::Lookup('a').length == 2, "lower case 'a' works (= '.-')");
    Check(morse::CountUnits(nullptr) == 0, "CountUnits(nullptr) is 0");
    Check(morse::CountUnits("") == 0, "CountUnits(\"\") is 0");
    Check(morse::CountUnits("???") == 0, "all-unsupported string is 0 units");

    printf("\n--- morse: out of range ArdfId ---\n");
    {
        const morse::ArdfId bad = static_cast<morse::ArdfId>(99);
        Check(morse::GetArdfIdText(bad) != nullptr, "GetArdfIdText(99) not null");
        Check(morse::GetArdfSlot(bad) <= 5, "GetArdfSlot(99) in range");
    }

    printf("\n--- keyer: misuse ---\n");
    {
        CwKeyer k;  // no tone generator attached
        k.Send("MOE");
        const float x = k.Tick();
        Check(x == 0.0f, "Tick() with no tone generator returns 0");
    }
    {
        ToneGenerator t; t.Initialize();
        CwKeyer k; k.SetToneGenerator(&t);
        k.Send(nullptr);
        Check(!k.IsBusy(), "Send(nullptr) does not start the keyer");
        k.Send("");
        for (int i = 0; i < 1000; ++i) k.Tick();
        Check(!k.IsBusy(), "empty string finishes promptly");
    }
    {
        ToneGenerator t; t.Initialize();
        CwKeyer k; k.SetToneGenerator(&t); k.SetWpm(15);
        // Longer than kMaxTextLength (32)
        std::string longtext(80, 'E');
        k.Send(longtext.c_str());
        int n = 0;
        while (k.IsBusy() && n < 25000*60) { k.Tick(); ++n; }
        Check(n < 25000*60, "over-long text still terminates");
        Check(n > 0, "over-long text was truncated, not rejected");
    }

    printf("\n--- keyer: WPM clamping ---\n");
    {
        ToneGenerator t; t.Initialize();
        CwKeyer k; k.SetToneGenerator(&t);
        k.SetWpm(0);   Check(k.GetWpm() >= 5,  "SetWpm(0) clamped up");
        k.SetWpm(200); Check(k.GetWpm() <= 40, "SetWpm(200) clamped down");
    }

    printf("\n--- keyer: envelope fits inside a dit (spec 10.3) ---\n");
    {
        // "envelope x2 < dit" must hold, upper limit about 60 WPM.
        for (uint8_t wpm : {15, 20, 30, 40}) {
            ToneGenerator t; t.Initialize();
            CwKeyer k; k.SetToneGenerator(&t); k.SetWpm(wpm);
            const uint32_t dit = 1200u*25000u/(k.GetWpm()*1000u);
            const bool ok = 2*125u < dit;   // 125 samples = 5ms envelope
            char buf[80];
            snprintf(buf,sizeof buf,"%u WPM: dit=%u samples > 2x envelope(250)", k.GetWpm(), dit);
            Check(ok, buf);
        }
    }

    printf("\n--- keyer: Stop mid-transmission ---\n");
    {
        ToneGenerator t; t.Initialize();
        CwKeyer k; k.SetToneGenerator(&t); k.SetWpm(15);
        k.Send("JL1HMT");
        for (int i = 0; i < 5000; ++i) k.Tick();
        Check(k.IsBusy(), "busy mid-string");
        k.Stop();
        k.Tick();
        Check(!k.IsBusy(), "Stop() clears busy immediately");
        // and the tone must decay, not cut hard
        float first = std::fabs(k.Tick());
        Check(first < 0.912f, "output does not jump to full scale after Stop");
    }

    printf("\n--- scheduler: no dependencies wired ---\n");
    {
        ArdfScheduler s;
        s.Start();               // no TxController, no keyer
        for (int i=0;i<100;++i) s.Tick();
        Check(true, "Tick() with nothing wired does not crash");
        s.Stop();
    }

    printf("\n--- scheduler: callsign overflow ---\n");
    {
        ArdfScheduler s;
        std::string longcall(200, 'X');
        s.SetCallsign(longcall.c_str());   // buffer is 16
        s.SetCallsign(nullptr);
        Check(true, "over-long / null callsign does not crash");
    }

    printf("\n--- scheduler: time wraps across cycles ---\n");
    {
        ArdfScheduler s;
        ToneGenerator t; t.Initialize();
        CwKeyer k; k.SetToneGenerator(&t);
        s.SetCwKeyer(&k);
        s.SetArdfId(morse::ArdfId::kMoe);   // slot 1: 0-60s
        s.SetCallsign("JL1HMT");
        g_fake_time_us = 0;
        s.Start();
        // Several cycles later the countdown must still be sane.
        bool ok = true;
        for (int cyc = 0; cyc < 5; ++cyc) {
            g_fake_time_us = ((int64_t)cyc*300 + 90) * 1000000;  // 1:30 into each
            const uint32_t left = s.GetSecondsToNextSlot();
            if (left != 300-90) ok = false;
        }
        Check(ok, "countdown correct across 5 cycles");

        // And a long run - 2 hours in, per spec 10.6
        g_fake_time_us = (int64_t)(7200 + 30) * 1000000;
        const uint32_t left = s.GetSecondsToNextSlot();
        Check(left == 0 || left <= 300, "countdown sane after 2 hours");
    }

    printf("\n--- tone: amplitude never exceeds the A2A limit ---\n");
    {
        ToneGenerator t; t.Initialize();
        t.SetKey(true);
        float peak = 0.0f;
        for (int i = 0; i < 25000; ++i) peak = std::fmax(peak, std::fabs(t.Next()));
        Check(peak <= 0.9121f, "peak stays within kXGainA2a");
    }

    printf("\n--- tone: carrier keying mode reaches -1 (duty 0) ---\n");
    {
        ToneGenerator t; t.Initialize();
        t.SetKeyingMode(ToneGenerator::KeyingMode::kCarrierKeying);
        t.SetKey(false);
        float v = 0.0f;
        for (int i = 0; i < 500; ++i) v = t.Next();
        Check(std::fabs(v + 1.0f) < 0.01f, "key-up settles at x = -1");
    }

    printf("\n===== %s (%d failure%s) =====\n",
           fails?"FAILURES":"ALL PASS", fails, fails==1?"":"s");
    return fails != 0;
}
