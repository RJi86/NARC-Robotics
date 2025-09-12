#pragma once
#include <Arduino.h>
#include "R_Multiplexer/analog_multi.h"

// Declarations only — NO parentheses here
extern analog_multi_t IRcluster_1;
extern analog_multi_t IRcluster_2;
extern analog_multi_t IRcluster_3;

uint16_t readIrSensor(uint8_t idx);