#pragma once
#include <Arduino.h>
#include <math.h>
#include "Light_pins.h" // provides LIGHT_SENSOR_COUNT

struct point_t { float x; float y; };

class light_cluster_t {
public:
light_cluster_t();
light_cluster_t(uint8_t min, uint8_t max = 0);

float angle() const; // Center-of-mass angle in degrees [0,360)
uint8_t count() const; // Number of sensors in cluster (wrap‑aware)
point_t point() const; // Unit vector at that angle

uint8_t get_min() const; void set_min(uint8_t min);
uint8_t get_max() const; void set_max(uint8_t max);

private:
uint8_t m_min;
uint8_t m_max;
};
