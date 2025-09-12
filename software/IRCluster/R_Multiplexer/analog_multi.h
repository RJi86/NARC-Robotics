#pragma once
#include <Arduino.h>
#include "analog.h"

class analog_multi_t : public analog_t {
public:
  analog_multi_t(uint8_t pin, uint8_t addr_pin_a, uint8_t addr_pin_b, uint8_t addr_pin_c)
  : analog_t(pin),
    m_addr_pin_a(addr_pin_a),
    m_addr_pin_b(addr_pin_b),
    m_addr_pin_c(addr_pin_c) 
  {
    pinMode(m_addr_pin_a, OUTPUT);
    pinMode(m_addr_pin_b, OUTPUT);
    pinMode(m_addr_pin_c, OUTPUT);
    digitalWrite(m_addr_pin_a, LOW);
    digitalWrite(m_addr_pin_b, LOW);
    digitalWrite(m_addr_pin_c, LOW);
  }

  inline void set_address(uint8_t address) {
    m_selected_addr = address & 0x07;
    digitalWrite(m_addr_pin_a, (m_selected_addr & 0b001) ? HIGH : LOW);
    digitalWrite(m_addr_pin_b, (m_selected_addr & 0b010) ? HIGH : LOW);
    digitalWrite(m_addr_pin_c, (m_selected_addr & 0b100) ? HIGH : LOW);
  }

  inline uint8_t get_address() const { return m_selected_addr; }

  inline uint16_t read(uint8_t address) {
    set_address(address);
    delayMicroseconds(5);
    return analog_t::read();
  }

private:
  uint8_t m_addr_pin_a, m_addr_pin_b, m_addr_pin_c;
  uint8_t m_selected_addr = 0;
};

// Back-compat alias so any code using `analog_multi` still works
using analog_multi = analog_multi_t;