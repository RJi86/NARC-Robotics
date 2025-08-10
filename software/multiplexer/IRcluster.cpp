#include <Arduino.h>
#include "IRcluster.h"

#define PI = 3.14159265
float sensor_angles[TSSP40_COUNT];

// Set up angles in radians (15° spacing assumed)
void setup_sensor_angles() {
  for (int i = 0; i < TSSP40_COUNT; ++i) {
    sensor_angles[i] = (i * 15.0 * PI / 180.0);
  }
}

ircluster::ircluster(sig_8 min, sig_8 max)
  : min1(min % TSSP40_COUNT), max1(max % TSSP40_COUNT) {}

float ircluster::angletheta() const {
  int i = min1;
  float sum = 0;
  int count_val = 0;

  do {
    float angle = sensor_angles[i];
    sum += angle;
    count_val++;
    i = (i + 1) % TSSP40_COUNT;
  } while (i != (max1 + 1) % TSSP40_COUNT);

  float avg = sum / count_val;
  if (avg < 0) avg += 2 * PI;
  return avg;
}

const ssize_t ircluster::count() {
  if (min1 <= max1)
    return max1 - min1 + 1;
  else
    return TSSP40_COUNT - min1 + max1 + 1;
}


sig_8 ircluster::get_min() const {
  return min1;
}

sig_8 ircluster::get_max() const {
  return max1;
}

void ircluster::set_min(sig_8 min) {
  min1 = min % TSSP40_COUNT;
}

void ircluster::set_max(sig_8 max) {
  max1 = max % TSSP40_COUNT;
}
