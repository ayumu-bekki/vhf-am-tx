// Host test: settings are saved in the background, and only when changed.
//
// The point is latency. A flash commit takes tens of milliseconds; if it runs
// on the caller's thread, the UI stalls on every mode press. The stub NVS here
// sleeps 50ms in nvs_commit() to stand in for that.
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include "transmitter_settings.h"

using namespace transmitter_system;
int64_t g_fake_time_us = 0;

static double MsSince(std::chrono::steady_clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - t0)
      .count();
}
static void SleepMs(int ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main() {
  int failures = 0;
  auto check = [&](bool ok, const char* what) {
    printf("  %-62s %s\n", what, ok ? "OK" : "MISMATCH");
    if (!ok) ++failures;
  };

  // esp_timer is faked for the other host tests; the settings use it for the
  // hold-off, so advance it alongside real time here.
  auto advance = [&](int ms) { g_fake_time_us += (int64_t)ms * 1000; };

  printf("--- forced save returns at once (mode press) ---\n");
  {
    TransmitterSettings s;
    s.Load();
    s.StartWriter();
    g_nvs.commits = 0;

    s.SetCwWpm(s.GetCwWpm() == 20 ? 21 : 20);
    auto t0 = std::chrono::steady_clock::now();
    s.Tick(/*force=*/true);
    const double ms = MsSince(t0);
    printf("  Tick(force) took %.2f ms (commit itself takes 50 ms)\n", ms);
    check(ms < 10.0, "caller is not blocked by the flash write");

    SleepMs(300);
    check(g_nvs.commits == 1, "the write still happened, on the background task");
    check(!s.IsDirty(), "dirty flag cleared afterwards");
  }

  printf("\n--- nothing is written when nothing changed ---\n");
  {
    TransmitterSettings s;
    s.Load();
    s.StartWriter();
    g_nvs.commits = 0;
    s.SetFrequencyHz(s.GetFrequencyHz());
    s.SetCwWpm(s.GetCwWpm());
    s.Tick(true);
    advance(10000);
    SleepMs(300);
    check(g_nvs.commits == 0, "setting the same values causes no commit");
  }

  printf("\n--- deferred save waits for the hold-off ---\n");
  {
    TransmitterSettings s;
    s.Load();
    s.StartWriter();
    g_nvs.commits = 0;
    s.SetCwWpm(10);
    SleepMs(250);
    check(g_nvs.commits == 0, "no commit before the 3 s hold-off");
    advance(hardware_config::kNvsWriteDelayMs + 100);
    SleepMs(400);
    check(g_nvs.commits == 1, "one commit after the hold-off");
  }

  printf("\n--- a change during a write is not lost ---\n");
  {
    TransmitterSettings s;
    s.Load();
    s.StartWriter();
    g_nvs.commits = 0;
    s.SetCwWpm(11);
    s.Tick(true);
    SleepMs(20);  // writer is now inside the 50 ms commit
    s.SetCwWpm(12);
    advance(hardware_config::kNvsWriteDelayMs + 100);
    SleepMs(500);
    check(g_nvs.commits == 2, "second change got its own commit");
    check(g_nvs.kv["cw_wpm"] == "12", "stored value is the latest one");
  }

  printf("\n--- callsign: only 6 characters of A-Z / 0-9 are accepted ---\n");
  {
    check(TransmitterSettings::IsValidCallsign("JL1HMT"), "JL1HMT accepted");
    check(!TransmitterSettings::IsValidCallsign("JA1AB"), "too short rejected");
    check(!TransmitterSettings::IsValidCallsign("JA1ABCD"), "too long rejected");
    check(!TransmitterSettings::IsValidCallsign("ja1abc"), "lower case rejected");
    check(!TransmitterSettings::IsValidCallsign(nullptr), "null rejected");
    TransmitterSettings s;
    check(!s.HasCallsign(), "no built-in default callsign");
    s.SetCallsign("JA1AB");
    check(!s.HasCallsign(), "invalid set is ignored");
    s.SetCallsign("JA1ABC");
    check(std::string(s.GetCallsign()) == "JA1ABC", "valid set is taken");
  }

  printf("\n--- callsign: loaded from NVS at boot ---\n");
  {
    {
      std::lock_guard<std::mutex> l(g_nvs.m);
      g_nvs.kv["call"] = "JA1ABC";
    }
    TransmitterSettings s;
    s.Load();
    check(std::string(s.GetCallsign()) == "JA1ABC", "saved callsign is used");

    {
      std::lock_guard<std::mutex> l(g_nvs.m);
      g_nvs.kv["call"] = "ja1";
    }
    TransmitterSettings bad;
    bad.Load();
    check(!bad.HasCallsign(), "invalid saved callsign leaves it unset");

    {
      std::lock_guard<std::mutex> l(g_nvs.m);
      g_nvs.kv.erase("call");
    }
    TransmitterSettings none;
    none.Load();
    check(!none.HasCallsign(), "nothing saved leaves it unset");
  }

  printf("\n--- power / emission: stored values, out-of-range falls back (P5) ---\n");
  {
    auto put = [](const char* k, const char* v) {
      std::lock_guard<std::mutex> l(g_nvs.m);
      g_nvs.kv[k] = v;
    };
    {
      TransmitterSettings s;
      s.Load();
      check(s.GetPower() == hardware_config::kPowerDefault, "no key: power default");
      check(s.GetEmission() == Emission::kA2a, "no key: emission A2A");
    }
    put("power", "2");
    put("emission", "1");
    {
      TransmitterSettings s;
      s.Load();
      check(s.GetPower() == 2 && s.GetEmission() == Emission::kA1a,
            "valid values are restored");
    }
    put("power", "3");
    put("emission", "2");
    put("mode", "9");
    {
      TransmitterSettings s;
      s.Load();
      check(s.GetPower() == hardware_config::kPowerDefault, "power 3 -> default");
      check(s.GetEmission() == Emission::kA2a, "emission 2 -> A2A");
      check(g_nvs.kv.count("mode") == 0, "old mode key is erased, never restored");
    }
    put("power", "255");
    {
      TransmitterSettings s;
      s.Load();
      check(s.GetPower() == hardware_config::kPowerDefault, "power 255 -> default");
      s.SetPower(7);
      check(s.GetPower() == hardware_config::kPowerDefault, "SetPower(7) refused");
      s.SetPower(1);
      s.SetEmission(Emission::kA1a);
      s.Tick(true);
      check(g_nvs.kv["power"] == "1" && g_nvs.kv["emission"] == "1",
            "changes are written under power / emission");
    }
  }

  printf("\n===== %s (%d failure%s) =====\n", failures ? "FAILURES" : "ALL PASS",
         failures, failures == 1 ? "" : "s");
  return failures != 0;
}
