#pragma once
#include <Arduino.h>
#include "R_Multiplexer/analog_multi.h"
#include "R_Multiplexer/Pins.h"
#include "ClusterConfig.h"

class IRRing {
public:
  IRRing()
  : m0(MUX0_SIG, MUX0_S0, MUX0_S1, MUX0_S2)
  , m1(MUX1_SIG, MUX1_S0, MUX1_S1, MUX1_S2)
  , m2(MUX2_SIG, MUX2_S0, MUX2_S1, MUX2_S2) {}

  void scan(uint16_t out[clustercfg::kCount]) {
    for (uint8_t ch=0; ch<8; ++ch) out[ch]      = m0.read(ch);
    for (uint8_t ch=0; ch<8; ++ch) out[8  + ch] = m1.read(ch);
    for (uint8_t ch=0; ch<8; ++ch) out[16 + ch] = m2.read(ch);
  }
private:
  analog_multi_t m0, m1, m2;
};
