#pragma once
#include <Arduino.h>
#include "ClusterConfig.h"
#include "IRcluster/IRCluster.h"
#include "controllers/avg_angle.h"

struct BearingResult {
float bearing_rad = 0.0f; // 0..2pi
uint8_t min_idx = 0;
uint8_t max_idx = 0;
};

class BaseSensorController {
public:
// this has to do with camera stuff
};