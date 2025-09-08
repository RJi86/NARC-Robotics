#pragma once
#include <Arduino.h>
#include "analog.h"

class analog_multi_t : public analog_t {
public:
analog_multi_t(uint8_t pin, uint8_t s0, uint8_t s1, uint8_t s2)
: analog_t(pin), m_s0(s0), m_s1(s1), m_s2(s2) {
pinMode(m_s0, OUTPUT);
pinMode(m_s1, OUTPUT);
pinMode(m_s2, OUTPUT);
digitalWrite(m_s0, LOW);
digitalWrite(m_s1, LOW);
digitalWrite(m_s2, LOW);
}

uint16_t read(uint8_t address) {
set_address(address);
delayMicroseconds(1); // allow switches to settle
return analog_t::read();
}

void set_address(uint8_t address) {
m_addr = (address & 0x07);
digitalWrite(m_s0, (m_addr & 0x01) ? HIGH : LOW);
digitalWrite(m_s1, (m_addr & 0x02) ? HIGH : LOW);
digitalWrite(m_s2, (m_addr & 0x04) ? HIGH : LOW);
}

uint8_t get_address() const { return m_addr; }

private:
uint8_t m_addr = 0;
const uint8_t m_s0, m_s1, m_s2;
};
