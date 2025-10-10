#include <Arduino.h>
#include <math.h>
#include "LightCluster.h"
#include "ClusterConfig.h"

using namespace clustercfg;

namespace {
  struct Sct {
    float c[kCount];
    float s[kCount];
    bool  init = false;
    void ensure_init() {
      if (init) return;
      for (uint8_t i = 0; i < kCount; ++i) {
        float ang = (i * kDegStep) * (kPI / 180.0f);
        c[i] = cosf(ang);
        s[i] = sinf(ang);
      }
      init = true;
    }
  } table;
}

LightCluster::LightCluster(idx_t min, idx_t max) noexcept
: min1(wrap(min)), max1(wrap(max)) {}

size_t LightCluster::count() const noexcept {
  if (min1 <= max1) return static_cast<size_t>(max1 - min1 + 1);
  return static_cast<size_t>(kCount - min1 + max1 + 1);
}

float LightCluster::angletheta() const noexcept {
  table.ensure_init();
  float sumC = 0.0f, sumS = 0.0f;
  uint8_t i = min1;
  do {
    sumC += table.c[i];
    sumS += table.s[i];
    i = (i + 1) % kCount;
  } while (i != (uint8_t)((max1 + 1) % kCount));

  if (sumC == 0.0f && sumS == 0.0f) return 0.0f;
  float ang = atan2f(sumS, sumC);
  if (ang < 0) ang += 2.0f * kPI;
  return ang;
}

void LightCluster::set_min(idx_t min) noexcept { min1 = wrap(min); }
void LightCluster::set_max(idx_t max) noexcept { max1 = wrap(max); }
