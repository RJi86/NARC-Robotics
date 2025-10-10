#pragma once
#include <Arduino.h>
#include "controllers/BaseSensorController.h"
#include "controllers/angle_utils.h"

class VisionFusion {
public:
VisionFusion(BaseSensorController& ir, BaseSensorController& light)
: m_ir(ir), m_light(light) {}

void update(){ m_ir.update(); m_light.update(); }

BearingResult fused() const {
auto irB = m_ir.bearing();
auto ltB = m_light.bearing();
// if one has much higher confidence or angles disagree crazy, pick max
float d = fabsf(DirectionUtils::angdiff(irB.bearing_rad, ltB.bearing_rad));
if (irB.confidence > ltB.confidence * 1.3f || d > (60.0f * clustercfg::kPI/180.0f)) return irB;
if (ltB.confidence > irB.confidence * 1.3f) return ltB;
// else blend by confidences
float w = ltB.confidence / max(0.001f, (irB.confidence + ltB.confidence));
BearingResult r;
r.bearing_rad = DirectionUtils::mix_angle(irB.bearing_rad, ltB.bearing_rad, w);
r.confidence = max(irB.confidence, ltB.confidence);
r.min_idx = 0; r.max_idx = 0; // not meaningful after fusion
return r;
}

private:
BaseSensorController& m_ir;
BaseSensorController& m_light;
};
