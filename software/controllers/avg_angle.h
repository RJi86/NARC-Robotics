#pragma once
#include <Arduino.h>

template <size_t N>
struct avg_angle {
float buf[N]{}; size_t k=0; bool filled=false; float sum=0;
float step(float x){ sum += x - buf[k]; buf[k]=x; k=(k+1)%N; if(!filled&&k==0) filled=true; return sum/(filled?N:k); }
void reset(){ for(auto &v:buf) v=0; k=0; filled=false; sum=0; }
};

