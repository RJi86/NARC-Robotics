#include "IRclusterHW.h"
#include "R_Multiplexer/Pins.h"

analog_multi_t IRcluster_1(MUX0_SIG, MUX0_S0, MUX0_S1, MUX0_S2);
analog_multi_t IRcluster_2(MUX1_SIG, MUX1_S0, MUX1_S1, MUX1_S2);
analog_multi_t IRcluster_3(MUX2_SIG, MUX2_S0, MUX2_S1, MUX2_S2);

uint16_t readIrSensor(uint8_t idx) {
  const uint8_t cluster = idx / 8, ch = idx % 8;
  switch (cluster) {
    case 0: return IRcluster_1.read(ch);
    case 1: return IRcluster_2.read(ch);
    case 2: return IRcluster_3.read(ch);
    default: return 0;
  }
}