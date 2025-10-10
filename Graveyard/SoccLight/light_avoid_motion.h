#pragma once
return d;
}
static inline float deg_to_rad(float d) { return d * 0.01745329251994329577f; }
static inline float rad_to_deg(float r) { return r * 57.295779513082320876f; }

static uint8_t count_in_sector(uint32_t mask, float center_deg, float cone_deg) {
const float step = 360.0f / LIGHT_SENSOR_COUNT;
const int half = (int)lroundf((cone_deg * 0.5f) / step);
int cidx = (int)lroundf(center_deg / step);
cidx %= LIGHT_SENSOR_COUNT; if (cidx < 0) cidx += LIGHT_SENSOR_COUNT;
uint8_t hits = 0;
for (int k = -half; k <= half; ++k) {
int idx = (cidx + k) % LIGHT_SENSOR_COUNT; if (idx < 0) idx += LIGHT_SENSOR_COUNT;
if (bit_is_set_msb32(mask, (uint8_t)idx)) ++hits;
}
return hits;
}

static inline bool white_barrier_ahead(uint32_t mask) {
return count_in_sector(mask, 0.0f, WHITE_CONE_DEG) >= WHITE_MIN_HITS;
}

// 90° avoidance: move perpendicular to the white cluster direction.
// If white is on the left (relative to forward 0°), move right (+90°). If on the right, move left (−90°).
static inline float avoidance_theta_deg(uint16_t angle_cdeg) {
const float a = angle_cdeg / 100.0f; // 0..360
const float rel = norm180(a - 0.0f); // relative to front
return (rel >= 0.0f) ? a + 90.0f : a - 90.0f; // pick side away from white
}

// High-level: compute and issue a MotorClass command.
// base_pwm: 0..255 speed for translation. If barrier directly ahead → stop.
static inline void apply_light_avoid_motion(uint16_t angle_cdeg,
bool on_white,
uint32_t mask,
MotorClass& motors,
uint8_t base_pwm) {
if (white_barrier_ahead(mask)) {
motors.MoveDirection(0.0f, 0.0f); // hard stop
return;
}
if (on_white) {
float theta_deg = fmodf(avoidance_theta_deg(angle_cdeg), 360.0f);
if (theta_deg < 0) theta_deg += 360.0f;
const float theta_rad = deg_to_rad(theta_deg);
motors.MoveDirection(theta_rad, (float)base_pwm);
} else {
motors.MoveDirection(0.0f, (float)base_pwm); // drive straight forward
}
}
