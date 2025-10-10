#include "analog_multi.h"

analog_multi_t::analog_multi_t(const uint8_t pin, const uint8_t addr_pin_a, const uint8_t addr_pin_b, const uint8_t addr_pin_c)
	: analog_t(pin), m_addr_pin_a(addr_pin_a), m_addr_pin_b(addr_pin_b), m_addr_pin_c(addr_pin_c)
{
	pinMode(m_addr_pin_a, OUTPUT);
	pinMode(m_addr_pin_b, OUTPUT);
	pinMode(m_addr_pin_c, OUTPUT);

	digitalWrite(m_addr_pin_a, LOW);
	digitalWrite(m_addr_pin_b, LOW);
	digitalWrite(m_addr_pin_c, LOW);
}

uint16_t analog_multi_t::read(uint8_t address)
{
	set_address(address);
	delayMicroseconds(1);
	return analog_t::read();
}

void analog_multi_t::set_address(uint8_t address)
{
	m_selected_addr = address;

	digitalWrite(m_addr_pin_a, (m_selected_addr & 0b001) ? HIGH : LOW);
	digitalWrite(m_addr_pin_b, (m_selected_addr & 0b010) ? HIGH : LOW);
	digitalWrite(m_addr_pin_c, (m_selected_addr & 0b100) ? HIGH : LOW);
}

uint8_t analog_multi_t::get_address() const
{
	return m_selected_addr;
}
