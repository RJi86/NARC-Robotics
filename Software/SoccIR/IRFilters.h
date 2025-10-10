#pragma once
#include <Arduino.h>

struct DigitalFilter {
  int count = 0;
  int threshold_on = 3;
  int threshold_off = 1;
  int max_count = 5;
  bool state = false;

  bool process(bool input) {
    // Integrate input over time
    if (input) {
      if (count < max_count) count++;
    } else {
      if (count > 0) count--;
    }

    // Apply hysteresis logic
    if (!state && count >= threshold_on)
      state = true;
    else if (state && count <= threshold_off)
      state = false;

    return state;
  }
};