#pragma once
#include <Arduino.h>

namespace pins {
// 0-7
constexpr uint8_t MUX0_SIG = A0; //sig
constexpr uint8_t MUX0_S0 = 2; 
constexpr uint8_t MUX0_S1 = 3; 
constexpr uint8_t MUX0_S2 = 4; 
// 8-15
constexpr uint8_t MUX1_SIG = A1;
constexpr uint8_t MUX1_S0 = 5;
constexpr uint8_t MUX1_S1 = 6;
constexpr uint8_t MUX1_S2 = 7;

// 16-23
constexpr uint8_t MUX2_SIG = A2;
constexpr uint8_t MUX2_S0 = 8;
constexpr uint8_t MUX2_S1 = 9;
constexpr uint8_t MUX2_S2 = 10;
}