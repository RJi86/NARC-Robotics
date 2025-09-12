#include <Wire.h>
#include <Arduino.h>
#include "Light_pins.h"
#include "analog_multi_light.h"
#include "light_cluster.h"
#define DEBUG_SERIAL 1

#ifndef LIGHT_I2C_ADDRESS
#define LIGHT_I2C_ADDRESS 0x42
#endif
#ifndef LIGHT_CLUSTER_COUNT
#define LIGHT_CLUSTER_COUNT 4
#endif
#ifndef LIGHT_SENSOR_COUNT
#define LIGHT_SENSOR_COUNT 32
#endif
#ifndef LIGHT_MASK_BYTES
#define LIGHT_MASK_BYTES 4
#endif
#ifndef LDR1_SIG
#define LDR1_SIG A0
#define LDR1_A0 20
#define LDR1_A1 21
#define LDR1_A2 22
#endif
#ifndef LDR2_SIG
#define LDR2_SIG A1
#define LDR2_A0 23
#define LDR2_A1 26
#define LDR2_A2 27
#endif
#ifndef LDR3_SIG
#define LDR3_SIG A2
#define LDR3_A0 14
#define LDR3_A1 15
#define LDR3_A2 41
#endif
#ifndef LDR4_SIG
#define LDR4_SIG A3
#define LDR4_A0 38
#define LDR4_A1 39
#define LDR4_A2 40
#endif

// Forward declarations (needed by some cores before registering callbacks)
void i2c_request_event();
void i2c_receive_event(int bytes);

constexpr uint16_t THRESHOLD = 500;

// Instantiate 4 clusters (locked configuration)
analog_multi_t cluster1(LDR1_SIG, LDR1_A0, LDR1_A1, LDR1_A2);
analog_multi_t cluster2(LDR2_SIG, LDR2_A0, LDR2_A1, LDR2_A2);
analog_multi_t cluster3(LDR3_SIG, LDR3_A0, LDR3_A1, LDR3_A2);
analog_multi_t cluster4(LDR4_SIG, LDR4_A0, LDR4_A1, LDR4_A2);

// Public telemetry
static volatile bool on_white = false; // any cluster present
static volatile uint16_t angle = 0; // angle * 100 (0..36000)
static volatile uint32_t lightsensors = 0; // 32-bit mask (MSB first across 32 bits)

struct ClusterSpan { int16_t min = -1; int16_t max = -1; uint8_t len = 0; bool found = false; };

static uint16_t read_light_sensor(uint8_t sensor_index) {
const uint8_t cluster_index = sensor_index >> 3; // i/8
analog_multi_t* c = nullptr;
switch (cluster_index) {
case 0: c = &cluster1; break;
case 1: c = &cluster2; break;
case 2: c = &cluster3; break;
case 3: c = &cluster4; break;
default: return 0;
}
const uint8_t ch = sensor_index & 0x07; // i%8
return c->read(ch);
}

static ClusterSpan find_largest_cluster_circular(const bool* active, uint8_t n) {
// WHY: Correctly merges runs that straddle index 0 (wrap‑around)
uint8_t best_len = 0; int16_t best_end = -1; uint8_t cur_len = 0;
for (uint16_t i = 0; i < (uint16_t)(2 * n); ++i) {
if (active[i % n]) {
if (cur_len < n) ++cur_len; // clamp to N
if (cur_len > best_len) { best_len = cur_len; best_end = (int16_t)i; }
} else {
cur_len = 0;
}
}
ClusterSpan out;
if (best_len > 0) {
out.len = best_len; out.max = (int16_t)(best_end % n);
out.min = (int16_t)((out.max - best_len + 1 + n) % n); out.found = true;
}
return out;
}

void setup() {
Wire.begin(LIGHT_I2C_ADDRESS);
Wire.onRequest(i2c_request_event);
Wire.onReceive(i2c_receive_event);
#if DEBUG_SERIAL
Serial.begin(115200);
delay(500);
Serial.println(F("[socclight] boot"));
#endif
}

void loop() {

if (Serial.available()) {
char c = (char)Serial.read();
if (c == 'd') dump_raw(); // dumps all 32 raw readings
}
bool active[LIGHT_SENSOR_COUNT];
uint32_t mask = 0;

for (uint8_t i = 0; i < LIGHT_SENSOR_COUNT; ++i) {
const bool v = (read_light_sensor(i) > THRESHOLD);
active[i] = v;
// Pack 32 bits MSB-first: sensor 0 -> bit 31
mask |= (uint32_t)(v ? 1u : 0u) << (LIGHT_SENSOR_COUNT - 1 - i);
}

const ClusterSpan best = find_largest_cluster_circular(active, LIGHT_SENSOR_COUNT);
if (best.found) {
light_cluster_t cluster; cluster.set_min((uint8_t)best.min); cluster.set_max((uint8_t)best.max);
angle = (uint16_t)(cluster.angle() * 100.0f + 0.5f);
on_white = true;
} else { angle = 0; on_white = false; }
lightsensors = mask;

#if DEBUG_SERIAL
static uint32_t lastPrint = 0;
if (millis() - lastPrint > 500) {
lastPrint = millis();
Serial.print(F("ang="));
Serial.print(angle / 100.0f);
Serial.print(F(" white="));
Serial.print(on_white ? 1 : 0);
Serial.print(F(" mask=0x"));
Serial.println(lightsensors, HEX);
}
#endif
}

void i2c_request_event() {
Wire.write((uint8_t)((angle >> 8) & 0xFF));
Wire.write((uint8_t)(angle & 0xFF));
Wire.write((uint8_t)on_white);
// Write 4 bytes MSB-first for 32 sensors
for (int8_t b = LIGHT_MASK_BYTES - 1; b >= 0; --b) {
Wire.write((uint8_t)((lightsensors >> (8 * b)) & 0xFF));
}
}

void i2c_receive_event(int /*bytes*/) {
// No-op; reserved for future commands (eg. set threshold)
}
static void dump_raw() {
Serial.println(F("idx:val ..."));
for (uint8_t i = 0; i < LIGHT_SENSOR_COUNT; ++i) {
uint16_t v = read_light_sensor(i);
Serial.print(i); Serial.print(':'); Serial.print(v);
Serial.print((i % 8 == 7) ? '\n' : ' ');
}
}


