#pragma once

#include <Arduino.h>

class analog_t
{
public:
	analog_t(const uint8_t pin);

	uint16_t read() const;
private:
	const uint8_t m_pin;
};