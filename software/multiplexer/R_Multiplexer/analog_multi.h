#pragma once

#include "analog.h"

class analog_multi_t : public analog_t
{
public:
	analog_multi_t(const uint8_t pin, const uint8_t addr_pin_a, const uint8_t addr_pin_b, const uint8_t addr_pin_c);

	uint16_t read(uint8_t address);

	void set_address(uint8_t address);
	uint8_t get_address() const;
private:
	uint8_t m_selected_addr = 0b000;
	const uint8_t m_addr_pin_a, m_addr_pin_b, m_addr_pin_c;
};