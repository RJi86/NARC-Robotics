#pragma once
#include <Arduino.h>

#ifndef IRcluster_h
#define IRcluster_h



#define TSSP40_COUNT 24
typedef uint8_t sig_8;

class ircluster {
public:
    ircluster(sig_8 min, sig_8 max = 0);

    float angletheta() const;
    ssize_t count() const;

    sig_8 get_min() const;
    sig_8 get_max() const;

    void set_min(sig_8 min);
    void set_max(sig_8 max);

private:
    sig_8 min1;
    sig_8 max1;
};
