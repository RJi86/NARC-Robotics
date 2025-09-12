#pragma once
#include <Arduino.h>

namespace clustercfg {
  constexpr uint8_t kCount = 24;     // TSSP40_COUNT
  constexpr float    kDegStep = 15.0; // 24 sensors -> 360/24 = 15°
  constexpr float    kPI = 3.14159265358979323846f;
}