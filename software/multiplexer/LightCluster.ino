#include <Wire.h>
#include "Robot.h"
#include "LightCluster.h"
# define THRESHOLD 120// up to changing
robot_state_t state(11, 10);

analog_multi_t LightCluster_1(PIN_A0, PIN_PD0, PIN_PD1, PIN_PD2);
analog_multi_t LightCluster_2(PIN_PC1, PIN_PD3, PIN_PD4, PIN_PD5);
analog_multi_t LightCluster_3(PIN_PC2, PIN_PD6, PIN_PD7, PIN_PB0);
bool has_ball = false;
uint16_t angle = 0;
uint32_t tssps = 0;
unit_16_t readLightSensors (unit8_t counter)
{
uint8_t counter1 = (uint8_t)((float) counter / 8.0);
         switch (counter1) {
    case 1:
      value = IRcluster_1.read();
      break;
    case 2:
      value = IRcluster_2.read();
      break;
    case 3:
      value = IRcluster_3.read();
      break;  
   default: state.set_state(STATE_ERROR); return 0;
  }
 
  return value;
}//// toooooooo testttt
#include <Arduino.h>
#include "ClusterConfig.h"
#include "hw/LightRing.h"
#include "math/IRCluster.h" // reuse same cluster math for lights

LightRing lring;
uint16_t l_samples[clustercfg::kCount];

void setup(){
Serial.begin(115200); while(!Serial){}
Serial.println(F("Light cluster online"));
}

void loop(){
lring.scan(l_samples);
IRCluster c = IRCluster::dominant(l_samples, 0.6f);
float theta = c.angletheta();
Serial.print(F("LIGHT theta(deg)="));
Serial.print(theta * 180.0f / clustercfg::kPI, 1);
Serial.print(F(" cluster["));
Serial.print(c.get_min()); Serial.print(',');
Serial.print(c.get_max()); Serial.print(',');
Serial.print(c.count()); Serial.println(']');
delay(20);
}
