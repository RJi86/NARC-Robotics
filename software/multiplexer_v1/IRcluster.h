#pragma once
#include <Arduino.h>
#include "ClusterConfig.h"

class IRCluster {
public:
using idx_t = uint8_t;

IRCluster(idx_t min, idx_t max = 0) noexcept
: min1(wrap(min)), max1(wrap(max)) {}

size_t count() const noexcept {
if (min1 <= max1) return static_cast<size_t>(max1 - min1 + 1);
return static_cast<size_t>(clustercfg::kCount - min1 + max1 + 1);
}

float angletheta() const noexcept {
ensure_table();
float sumC = 0.0f, sumS = 0.0f;
uint8_t i = min1;
do {
sumC += tableC[i];
sumS += tableS[i];
i = static_cast<uint8_t>((i + 1) % clustercfg::kCount);
} while (i != static_cast<uint8_t>((max1 + 1) % clustercfg::kCount));

if (sumC == 0.0f && sumS == 0.0f) return 0.0f;
float ang = atan2f(sumS, sumC);
if (ang < 0) ang += 2.0f * clustercfg::kPI;
return ang;
}

idx_t get_min() const noexcept { return min1; }
idx_t get_max() const noexcept { return max1; }
void set_min(idx_t v) noexcept { min1 = wrap(v); }
void set_max(idx_t v) noexcept { max1 = wrap(v); }

// Build a cluster around the strongest reading using a simple threshold ratio.
static IRCluster dominant(const uint16_t *s, float ratio = 0.6f) {
// why: robust to noise; ratio keeps cluster contiguous around the true peak
const uint8_t N = clustercfg::kCount;
uint16_t peakVal = 0; uint8_t peakIdx = 0;
for (uint8_t i = 0; i < N; ++i) if (s[i] > peakVal) { peakVal = s[i]; peakIdx = i; }
const uint16_t thr = static_cast<uint16_t>(peakVal * ratio);

// expand to both sides while samples >= thr
auto prev = [&](uint8_t i){ return static_cast<uint8_t>((i + N - 1) % N); };
auto next = [&](uint8_t i){ return static_cast<uint8_t>((i + 1) % N); };

uint8_t lo = peakIdx, hi = peakIdx;
while (s[prev(lo)] >= thr) lo = prev(lo);
while (s[next(hi)] >= thr) hi = next(hi);
return IRCluster(lo, hi);
}

private:
static idx_t wrap(int v) noexcept {
return static_cast<idx_t>((v % clustercfg::kCount + clustercfg::kCount) % clustercfg::kCount);
}

static void ensure_table() {
if (tableInit) return;
for (uint8_t i = 0; i < clustercfg::kCount; ++i) {
const float ang = (i * clustercfg::kDegStep) * (clustercfg::kPI / 180.0f);
tableC[i] = cosf(ang);
tableS[i] = sinf(ang);
}
tableInit = true;
}

idx_t min1, max1;
static float tableC[clustercfg::kCount];
static float tableS[clustercfg::kCount];
static bool tableInit;
};