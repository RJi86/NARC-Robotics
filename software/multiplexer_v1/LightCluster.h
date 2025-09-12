#pragma once
#include <Arduino.h>
#include "drivers/analog_multi.h"
#include "Pins.h"
#include "ClusterConfig.h"

class LightRing {
public:
LightRing()
: m0(pins::LMUX0_SIG, pins::LMUX0_S0, pins::LMUX0_S1, pins::LMUX0_S2)
, m1(pins::LMUX1_SIG, pins::LMUX1_S0, pins::LMUX1_S1, pins::LMUX1_S2)
, m2(pins::LMUX2_SIG, pins::LMUX2_S0, pins::LMUX2_S1, pins::LMUX2_S2) {}

void scan(uint16_t out[clustercfg::kCount]) {
for (uint8_t ch = 0; ch < 8; ++ch) out[ch] = m0.read(ch);
for (uint8_t ch = 0; ch < 8; ++ch) out[8 + ch] = m1.read(ch);
for (uint8_t ch = 0; ch < 8; ++ch) out[16 + ch] = m2.read(ch);
}
private:
analog_multi_t m0, m1, m2;
};

