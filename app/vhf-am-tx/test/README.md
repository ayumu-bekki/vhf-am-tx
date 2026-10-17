# Host tests

Pure-logic tests that run on the development machine:

```sh
./run_tests.sh
```

## Why host tests

The CW keyer and the ARDF scheduler are driven by a tick count and a clock,
with no peripheral access in the logic itself. On hardware a single ARDF cycle
takes five minutes and the interesting moments are the slot boundaries, so the
clock is faked here (`g_fake_time_us`) and a full cycle runs in milliseconds.

The transmit sequencing test is the important one: spec §8.3 requires the duty
to ramp down *before* the carrier is cut, because reversing that dumps 1.5W
instantly, splatters the sidebands across tens of kHz, and leaves the
modulation node floating at 3.6V (the source follower cannot sink current).
`fakes_trace.cc` records every hardware action so the ordering can be asserted
rather than assumed.

## What these do not cover

Anything touching real peripherals — LEDC, ADC, I2C — and all ISR timing.
Those need the board; see plan.md §7.4 (measured ISR times) and §12 (test
modes). The `stub/` directory holds just enough of the ESP-IDF headers to
compile the logic, with hardware calls stubbed to fail; `fakes.cc` and
`fakes_trace.cc` supply working stand-ins for `Modulator` and `Si5351A`.
