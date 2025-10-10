#include <Arduino.h>

/*
Minimal diagnostic (no structs, no headers)
- 4 muxes × 8 channels = 32 readings
- strongest channel -> angle
- Serial @115200 prints every 500 ms
- Press 'd' to dump all 32 values
*/

// ===== EDIT PINS FOR YOUR BOARD =====
#define LDR_ACTIVE_HIGH 1   // set to 0 if voltage DROPS in light

// SIG pins MUST be analog-capable (A0..A3 are safest)
const uint8_t sig[4] = { A0, A1, A2, A3 };

// Select pins per cluster (A0/A1/A2 of the mux)
const uint8_t selA0[4] = { 20, 23, 14, 38 };
const uint8_t selA1[4] = { 21, 26, 15, 39 };
const uint8_t selA2[4] = { 22, 27, 41, 40 };

// Bit order mapping (if channels look scrambled, change this permutation)
#define MUX_BITL 0  // LSB bit -> selA0[*]
#define MUX_BITM 1 // mid bit -> selA1[*]
#define MUX_BITH 2  // MSB bit -> selA2[*]
// ====================================

static const uint8_t N = 32; // total channels
static uint16_t vals[N];

static inline void set_channel(uint8_t cluster, uint8_t ch) {
  ch &= 7;
  const uint8_t b0 = (ch >> 0) & 1;
  const uint8_t b1 = (ch >> 1) & 1;
  const uint8_t b2 = (ch >> 2) & 1;
  const uint8_t B[3] = { b0, b1, b2 };
  digitalWrite(selA0[cluster], B[MUX_BITL]);
  digitalWrite(selA1[cluster], B[MUX_BITM]);
  digitalWrite(selA2[cluster], B[MUX_BITH]);
}

static inline uint16_t read_channel(uint8_t cluster, uint8_t ch) {
  set_channel(cluster, ch);
  (void)analogRead(sig[cluster]);  // throw-away sample
  delayMicroseconds(50);           // settle
  return (uint16_t)analogRead(sig[cluster]);
}

void setup() {
  // Select pins as outputsSerial.begin(115200);
  while (!Serial && millis() < 4000) {}
  Serial.println("[adc sanity]");
  for (uint8_t c = 0; c < 4; ++c) {
    pinMode(sig[c], OUTPUT);
    pinMode(selA0[c], OUTPUT);
    pinMode(selA1[c], OUTPUT);
    pinMode(selA2[c], OUTPUT);
  }
#if defined(__IMXRT1062__) || defined(TEENSYDUINO)
  analogReadResolution(12);
  analogReadAveraging(4);
#endif
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 4000) { } // wait up to 4s
  Serial.println(F("[simple] boot"));
}

void loop() {
  // heartbeatint 
  int v0 = analogRead(A0);
  int v1 = analogRead(A1);
  int v2 = analogRead(A2);
  int v3 = analogRead(A3);
  Serial.print("A0..A3="); Serial.print(v0); Serial.print(',');
  Serial.print(v1); Serial.print(','); Serial.print(v2); Serial.print(',');
  Serial.println(v3);
  delay(300);
  static uint32_t tBlink=0; static bool led=false;
  if (millis()-tBlink > 500) { tBlink = millis(); led = !led; digitalWrite(LED_BUILTIN, led); }

  // sample 32 channels
  for (uint8_t i=0; i<8; ++i)  vals[i]      = read_channel(0, i);
  for (uint8_t i=0; i<8; ++i)  vals[8+i]    = read_channel(1, i);
  for (uint8_t i=0; i<8; ++i)  vals[16+i]   = read_channel(2, i);
  for (uint8_t i=0; i<8; ++i)  vals[24+i]   = read_channel(3, i);

  // pick dominant
  uint8_t bestIdx = 0; uint16_t bestVal = vals[0];
#if LDR_ACTIVE_HIGH
  for (uint8_t i=1; i<N; ++i) if (vals[i] > bestVal) { bestVal = vals[i]; bestIdx = i; }
#else
  for (uint8_t i=1; i<N; ++i) if (vals[i] < bestVal) { bestVal = vals[i]; bestIdx = i; }
#endif

  const float angle = bestIdx * (360.0f / N);

  static uint32_t tPrint=0;
  if (millis()-tPrint > 500) {
    tPrint = millis();
    Serial.print(F("idx="));   Serial.print(bestIdx);
    Serial.print(F(" angle="));Serial.print(angle, 2);
    Serial.print(F(" val="));  Serial.print(bestVal);
    Serial.println();
  }

  // press 'd' to dump raw 32 values
  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'd') {
      for (uint8_t i=0; i<N; ++i) {
        Serial.print(vals[i]); Serial.print((i%8==7)?'\n':' ');
      }
      Serial.println(F("---"));
    }
  }
}
