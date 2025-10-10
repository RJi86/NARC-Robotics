#include <Arduino.h>
#include "ClusterConfig.h"
#include "hw/IRRing.h"
#include "math/IRCluster.h"

IRRing g_ring;
uint16_t g_samples[clustercfg::kCount];

// Optional smoothing to stabilize readings
template <size_t N>
struct SMA {
float buf[N]{}; size_t k=0; bool filled=false; float sum=0;
float step(float x){ sum += x - buf[k]; buf[k]=x; k=(k+1)%N; if(!filled&&k==0) filled=true; return sum/(filled?N:k); }
};
SMA<4> g_sma; // 4-sample moving average of angle

static float wrap2pi(float a){
const float T = 2.0f * clustercfg::kPI;
while (a < 0) a += T; while (a >= T) a -= T; return a;
}

void setup(){
Serial.begin(115200);
while(!Serial) {}
Serial.println(F("IR ring online"));
}

void loop(){
// 1) Read all 24 sensors
g_ring.scan(g_samples);

// 2) Find dominant contiguous cluster around the brightest direction
IRCluster c = IRCluster::dominant(g_samples, 0.6f);

// 3) Compute robust bearing of that cluster
float theta = c.angletheta();
float theta_smooth = g_sma.step(theta);

// 4) Telemetry
Serial.print(F("cluster[min,max,count]=["));
Serial.print(c.get_min()); Serial.print(',');
Serial.print(c.get_max()); Serial.print(',');
Serial.print(c.count()); Serial.print("] theta(rad)=");
Serial.print(theta_smooth, 3);
Serial.print(F(" theta(deg)="));
Serial.println(theta_smooth * 180.0f / clustercfg::kPI, 1);

delay(10); // ~100 Hz
}
