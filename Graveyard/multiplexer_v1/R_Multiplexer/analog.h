#pragma once
#include <Arduino.h>

class analog_t {
public:
explicit analog_t(uint8_t pin) : m_pin(pin) {}
uint16_t read() const { return analogRead(m_pin); }
private:
const uint8_t m_pin;
};