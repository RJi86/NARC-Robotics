#pragma once
#include <stdint.h>
#include "ClusterConfig.h"

class LightCluster {
public:
    using idx_t = uint8_t;

    LightCluster(idx_t min, idx_t max) noexcept;

    size_t count() const noexcept;
    float angletheta() const noexcept;

    void set_min(idx_t min) noexcept;
    void set_max(idx_t max) noexcept;

private:
    idx_t min1;
    idx_t max1;

    static constexpr idx_t wrap(idx_t i) noexcept {
        // kCount lives in namespace clustercfg (from ClusterConfig.h)
        return static_cast<idx_t>(i % clustercfg::kCount);
    }
};
