#pragma once
#include <Arduino.h>
#include "ClusterConfig.h"

struct DirectionUtils {
static float wrap2pi(float a){
const float T = 2.0f * clustercfg::kPI;
while (a < 0) a += T; while (a >= T) a -= T; return a;
}

static float angdiff(float a, float b){
float d = wrap2pi(a) - wrap2pi(b);
if (d > clustercfg::kPI) d -= 2.0f * clustercfg::kPI;
if (d < -clustercfg::kPI) d += 2.0f * clustercfg::kPI;
return d;
}
// basic linear angle interpolation, handles wrap at 0/2pi- also makes sure that everything is absolute value 
static float mix_angle(float a, float b, float w){
float ca = cosf(a), sa = sinf(a);
float cb = cosf(b), sb = sinf(b);
float cx = (1.0f - w) * ca + w * cb;
float sx = (1.0f - w) * sa + w * sb;
float r = atan2f(sx, cx);
if (r < 0) r += 2.0f * clustercfg::kPI;
return r;
}
};
