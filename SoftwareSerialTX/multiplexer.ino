#include <Arduino.h>
#include "analog_multi.h"

// --- Pin map  S0, S1, S2 are address lines
#define PIN_A00 A0
#define PIN_S00 2
#define PIN_S01 3
#define PIN_S02 4

#define PIN_A10 A1
#define PIN_S10 5
#define PIN_S11 6
#define PIN_S12 7

#define PIN_A20 A2
#define PIN_S20 8
#define PIN_S21 9
#define PIN_S22 10

analog_multi_t mux0(PIN_A00, PIN_S00, PIN_S01, PIN_S02);
analog_multi_t mux1(PIN_A10, PIN_S10, PIN_S11, PIN_S12);
analog_multi_t mux2(PIN_A20, PIN_S20, PIN_S21, PIN_S22);

analog_multi_t* muxes[]   = { &mux0, &mux1, &mux2 };// pointer array for easier use
const char*     muxNames[] = { "MUX0", "MUX1", "MUX2" };
const uint8_t   NUM_MUXES  = sizeof(muxes) / sizeof(muxes[0]);

void setup() {
  Serial.begin(115200);
  // If you add analog_multi_t::begin(), call it here for each mux:
  for (uint8_t m = 0; m < NUM_MUXES; ++m) muxes[m]->begin();
}

static uint16_t readStable(analog_multi_t& mux, uint8_t ch) {
  // Mask to 0..7 to be safe- i have no idea how you use the channels
  ch &= 0x07;
  // First read to settle ADC after channel switch
  (void)mux.read(ch);
  delayMicroseconds(8);
  return mux.read(ch);
}

void loop() {
  for (uint8_t m = 0; m < NUM_MUXES; ++m) {
    Serial.print(muxNames[m]); Serial.println(":");
    for (uint8_t ch = 0; ch < 8; ++ch) {
      uint16_t val = readStable(*muxes[m], ch);
      Serial.print(ch);
       Serial.println(val);
      delay(2); 
    }
  }
  Serial.println();
  delay(250);
}
