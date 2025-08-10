analog_multi_t mux0(PIN_A00, PIN_A01, PIN_A02, PIN_D00);
analog_multi_t mux1(PIN_A10, PIN_A11, PIN_A12, PIN_D10);
analog_multi_t mux2(PIN_A20, PIN_A21, PIN_A22, PIN_D20);
#define PIN_A00 A0     // Arduino analog input for MUX 0 (e.g., A0)
#define PIN_A01 2      // Address A (S0) for MUX 0
#define PIN_A02 3      // Address B (S1) for MUX 0
#define PIN_D00 4      // Address C (S2) for MUX 0

// MUX 1 (optional – comment out if unused)
#define PIN_A10 A1
#define PIN_A11 5
#define PIN_A12 6
#define PIN_D10 7

// MUX 2 (optional – comment out if unused)
#define PIN_A20 A2
#define PIN_A21 8
#define PIN_A22 9
#define PIN_D20 10
analog_multi_t* muxes[] = { &mux0, &mux1, &mux2 };
const char*     muxNames[] = { "MUX0", "MUX1", "MUX2" };
const uint8_t   NUM_MUXES = sizeof(muxes) / sizeof(muxes[0]);
#include <Arduino.h>
void loop() {
  // Iterate all muxes and read all 8 addresses (0..7)
  for (uint8_t m = 0; m < NUM_MUXES; ++m) {
    Serial.print(muxNames[m]);
    Serial.println(F(":"));
    for (uint8_t ch = 0; ch < 8; ++ch) {
      uint16_t val = muxes[m]->read(ch); // internally sets address, small us delay, then analog read
      Serial.print(F("  CH "));
      Serial.print(ch);
      Serial.print(F(" -> "));
      Serial.println(val);
      delay(2); // tiny pacing so the serial log is readable
    }
  }
  Serial.println();
  delay(250); 
}