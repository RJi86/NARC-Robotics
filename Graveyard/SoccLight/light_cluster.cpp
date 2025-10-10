#include "light_cluster.h"

// Fallbacks if counts are not provided by pins_config.h (keeps build robust)
#ifndef LIGHT_SENSOR_COUNT
#define LIGHT_SENSOR_COUNT 32
#endif

light_cluster_t::light_cluster_t() : m_min(0), m_max(0) {}
light_cluster_t::light_cluster_t(uint8_t min, uint8_t max) : m_min(min), m_max(max) {}

float light_cluster_t::angle() const {
// WHY: Average angles across wrap requires continuity fix.
int i = m_min;
float sum = 0.0f;
while (true) {
float a = i * (360.0f / LIGHT_SENSOR_COUNT);
if (m_min > m_max && i > m_max) a -= 360.0f;
sum += a;
if (i == m_max) break;
i = (i + 1) % LIGHT_SENSOR_COUNT;
}
float result = sum / (float)count();
if (result < 0.0f) result += 360.0f;
return result;
}

uint8_t light_cluster_t::count() const {
int min = m_min, max = m_max;
if (min > max) min = -(LIGHT_SENSOR_COUNT - min);
int c = max - min + 1;
if (c < 0) c = 0; if (c > LIGHT_SENSOR_COUNT) c = LIGHT_SENSOR_COUNT;
return (uint8_t)c;
}

point_t light_cluster_t::point() const {
float a = angle() * DEG_TO_RAD;
return { cosf(a), sinf(a) };
}

uint8_t light_cluster_t::get_min() const { return m_min; }
void light_cluster_t::set_min(uint8_t min) { m_min = (uint8_t)(min % LIGHT_SENSOR_COUNT); }
uint8_t light_cluster_t::get_max() const { return m_max; }
void light_cluster_t::set_max(uint8_t max) { m_max = (uint8_t)(max % LIGHT_SENSOR_COUNT); }