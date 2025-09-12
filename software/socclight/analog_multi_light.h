#pragma once
#include <Arduino.h>

// Minimal 8:1 analog multiplexer helper.
class analog_multi_t {
public:
analog_multi_t(uint8_t analog_pin, uint8_t s0, uint8_t s1, uint8_t s2)
: a_(analog_pin), s0_(s0), s1_(s1), s2_(s2) {
pinMode(s0_, OUTPUT);
pinMode(s1_, OUTPUT);
pinMode(s2_, OUTPUT);
}

uint16_t read(uint8_t channel) {
channel &= 0x07;
digitalWrite(s0_, (channel >> 0) & 1);
digitalWrite(s1_, (channel >> 1) & 1);
digitalWrite(s2_, (channel >> 2) & 1);
// WHY: Allow mux to settle; first ADC read can be stale after channel switch.
(void)analogRead(a_);
delayMicroseconds(50);
return (uint16_t)analogRead(a_);
}

private:
uint8_t a_;
uint8_t s0_, s1_, s2_;
};
