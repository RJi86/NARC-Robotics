#include <Arduino.h>
#include "ClusterConfig.h"
#include "IRRing.h"

// Print all 24 values in a stable layout so you can see if channels move
IRRing ring;
uint16_t v[clustercfg::kCount];

void setup() {
  Serial.begin(115200);

  while(!Serial){}
  Serial.println(F("Mux self-test: watching all 24 channels"));
}

void loop() {
  ring.scan(v);

  // Print as three rows of 8 with indexes
  Serial.println(F("----"));
  for (uint8_t row=0; row<3; ++row) {
    uint8_t base = row*8;
    for (uint8_t i=0;i<8;++i){
      uint8_t idx = base+i;
      if (idx < 10) Serial.print('0');
      Serial.print(idx); Serial.print(':');
      Serial.print(v[idx]); Serial.print(' ');
    }
    Serial.println();
  }

  // Show the max index (should move as you move the ball)
  uint16_t peak=0; uint8_t pidx=0;
  for (uint8_t i=0;i<clustercfg::kCount;++i){ if (v[i]>peak){ peak=v[i]; pidx=i; } }
  Serial.print(F("peak idx=")); Serial.print(pidx);
  Serial.print(F(" val=")); Serial.println(peak);

  delay(80);
}
