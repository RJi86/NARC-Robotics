#pragma once
#include <Arduino.h>
#include "IRcluster/LightCluster.h"
#include "controllers/BaseSensorController.h"
#include "controllers/angle_utils.h"

class LightSensorController : public BaseSensorController {
public:
explicit LightSensorController(LightCluster& ring, float cluster_ratio = 0.6f)
: m_ring(ring), m_ratio(cluster_ratio) {}

void update() override {
m_ring.scan(m_raw);
// dynamic range compression (prevent single sensor saturation dominating)
for (uint8_t i=0;i<clustercfg::kCount;++i){
int32_t v = static_cast<int32_t>(m_raw[i]) - static_cast<int32_t>(m_base[i]);
v = v>0? v:0;
// soft knee: y = sqrt(v)
float y = sqrtf(static_cast<float>(v));
m_work[i] = static_cast<uint16_t>(y);
m_sum += m_work[i];
}
IRCluster c = IRCluster::dominant(m_work, m_ratio);
m_last.min_idx = c.get_min();
m_last.max_idx = c.get_max();
float theta = c.angletheta();
m_last.bearing_rad = m_sma.step(theta);
uint16_t peak = 0; for(uint8_t i=0;i<clustercfg::kCount;++i) peak = max<uint16_t>(peak, m_work[i]);
float mean = (m_sum>0) ? (float)m_sum / clustercfg::kCount : 0.0f; m_sum = 0;
float peakMean = (mean>0.f) ? constrain(peak/mean, 0.f, 10.f)/10.f : (peak>0?1.f:0.f);
float compact = constrain(8.0f / c.count(), 0.0f, 1.0f);
m_last.confidence = 0.65f*peakMean + 0.35f*compact;
}

BearingResult bearing() const override { return m_last; }
const uint16_t* samples() const override { return m_work; }

void calibrateAmbient(uint16_t reps = 32) override {
memset(m_base, 0, sizeof(m_base));
for (uint16_t r=0;r<reps;++r){
m_ring.scan(m_raw);
for (uint8_t i=0;i<clustercfg::kCount;++i) m_base[i] += m_raw[i];
}
for (uint8_t i=0;i<clustercfg::kCount;++i) m_base[i] = static_cast<uint16_t>(m_base[i]/reps);
}

private:
LightCluster & m_ring;
float m_ratio;
uint16_t m_raw[clustercfg::kCount]{};
uint16_t m_work[clustercfg::kCount]{};
uint16_t m_base[clustercfg::kCount]{};
SMA<4> m_sma;
mutable uint32_t m_sum=0;
BearingResult m_last{};
};
