#pragma once
#include <Arduino.h>
#include "drivers/analog_multi.h"
#include "Pins.h"
#include "ClusterConfig.h"

class IRRing {
public:
IRRing()
: m0(pins::MUX0_SIG, pins::MUX0_S0, pins::MUX0_S1, pins::MUX0_S2)
, m1(pins::MUX1_SIG, pins::MUX1_S0, pins::MUX1_S1, pins::MUX1_S2)
, m2(pins::MUX2_SIG, pins::MUX2_S0, pins::MUX2_S1, pins::MUX2_S2) {}

void scan(uint16_t out[clustercfg::kCount]) {
// indices 0-7
for (uint8_t ch = 0; ch < 8; ++ch)
out[ch] = m0.read(ch);
//indices 8-15
for (uint8_t ch = 0; ch < 8; ++ch)
out[8 + ch] = m1.read(ch);
//indices 16-23
for (uint8_t ch = 0; ch < 8; ++ch)
out[16 + ch] = m2.read(ch);
}

private:
analog_multi_t m0, m1, m2;
};
