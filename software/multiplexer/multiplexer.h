#pragma once
#include <Arduino.h>
#include "analog.h"
#include "analog_multi.h"
#ifndef Multiplexer_h
#define MultiPlexer_h
#define PIN_A00 0
#define PIN_A01 0
#define PIN_A02 0
#define PIN_D00 0
#define PIN_A10 0
#define PIN_A11 0
#define PIN_A12 0
#define PIN_D10 0
#define PIN_A20 0
#define PIN_A21 0
#define PIN_A22 0
#define PIN_D20 0


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