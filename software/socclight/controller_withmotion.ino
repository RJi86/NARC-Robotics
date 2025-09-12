#include <Arduino.h>
#include <Wire.h>
#include <MotorClass.h>
#include "light_avoid_motion.h"

// I²C address of the light-sensor board
#ifndef LIGHT_I2C_ADDRESS
#define LIGHT_I2C_ADDRESS 0x42
#endif

// Expecting packet: angle_hi, angle_lo, on_white, mask[4] (MSB first)
static bool read_light_packet(uint16_t& angle_cdeg, bool& on_white, uint32_t& mask) {
const uint8_t need = 7; // 2 + 1 + 4
Wire.requestFrom(LIGHT_I2C_ADDRESS, need);
if (Wire.available() < need) return false;
uint8_t hi = Wire.read();
uint8_t lo = Wire.read();
angle_cdeg = ((uint16_t)hi << 8) | lo;
on_white = Wire.read() != 0;
uint8_t b3 = Wire.read(); // MSB
uint8_t b2 = Wire.read();
uint8_t b1 = Wire.read();
uint8_t b0 = Wire.read(); // LSB
mask = ((uint32_t)b3 << 24) | ((uint32_t)b2 << 16) | ((uint32_t)b1 << 8) | b0;
return true;
}

MotorClass motors;

void setup() {
Wire.begin(); // master
Serial.begin(115200);
while (!Serial && millis() < 3000) {}
Serial.println(F("[controller] boot"));
}

void loop() {
uint16_t angle_cdeg = 0; bool on_white = false; uint32_t mask = 0;
if (read_light_packet(angle_cdeg, on_white, mask)) {
apply_light_avoid_motion(angle_cdeg, on_white, mask, motors, /*base_pwm*/ 160);

// Optional debug
static uint32_t t=0; if (millis()-t>500) { t=millis();
Serial.print(F("ang=")); Serial.print(angle_cdeg/100.0f);
Serial.print(F(" on=")); Serial.print(on_white);
Serial.print(F(" mask=0x")); Serial.println(mask, HEX);
}
}
delay(10);
}
