#!/bin/sh
# Host-side logic tests.
#
# These run on the development machine, not the target: the CW keyer and the
# ARDF scheduler are pure logic driven by a tick and a clock, and checking them
# on hardware would mean waiting five minutes per ARDF cycle. Time is faked, so
# a full cycle takes milliseconds.
#
# What they do NOT cover: anything touching real peripherals (LEDC, ADC, I2C)
# or the ISR timing. Those need the board - see plan.md §7.4 and §12.
set -e
cd "$(dirname "$0")"
MAIN=../main
CXX="${CXX:-g++}"
FLAGS="-std=c++17 -Wall -I stub -I $MAIN"

echo "=== modulation mapping / duty ceiling ==="
$CXX $FLAGS -o /tmp/test_modmap test_modmap.cc
/tmp/test_modmap

echo
echo "=== CW keyer / tone generator ==="
$CXX $FLAGS -o /tmp/test_keyer test_keyer.cc \
    $MAIN/morse.cc $MAIN/tone_generator.cc $MAIN/cw_keyer.cc
/tmp/test_keyer

echo
echo "=== ARDF scheduler ==="
$CXX $FLAGS -o /tmp/test_ardf test_ardf.cc fakes.cc \
    $MAIN/morse.cc $MAIN/tone_generator.cc $MAIN/cw_keyer.cc \
    $MAIN/ardf_scheduler.cc $MAIN/tx_controller.cc
/tmp/test_ardf

echo
echo "=== ARDF multi-cycle run ==="
$CXX $FLAGS -include algorithm -o /tmp/test_ardf_cycle test_ardf_cycle.cc fakes_trace.cc \
    $MAIN/morse.cc $MAIN/tone_generator.cc $MAIN/cw_keyer.cc \
    $MAIN/ardf_scheduler.cc $MAIN/tx_controller.cc
/tmp/test_ardf_cycle

echo
echo "=== transmit sequencing ==="
$CXX $FLAGS -include algorithm -o /tmp/test_txseq test_txseq.cc fakes_trace.cc \
    $MAIN/tx_controller.cc
/tmp/test_txseq

echo
echo "=== settings: background save ==="
$CXX $FLAGS -pthread -o /tmp/test_settings test_settings.cc \
    $MAIN/transmitter_settings.cc $MAIN/morse.cc
/tmp/test_settings

echo
echo "=== edge cases ==="
$CXX $FLAGS -o /tmp/test_edge test_edge.cc fakes.cc \
    $MAIN/morse.cc $MAIN/tone_generator.cc $MAIN/cw_keyer.cc \
    $MAIN/ardf_scheduler.cc $MAIN/tx_controller.cc
/tmp/test_edge

echo
echo "=== regressions ==="
$CXX $FLAGS -include algorithm -o /tmp/test_regression test_regression.cc fakes_trace.cc \
    $MAIN/morse.cc $MAIN/tone_generator.cc $MAIN/cw_keyer.cc \
    $MAIN/ardf_scheduler.cc $MAIN/tx_controller.cc
/tmp/test_regression

echo
echo "All host tests passed."
