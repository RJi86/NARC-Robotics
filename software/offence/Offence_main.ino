#include <Arduino.h>
#include "multiplexer/IRRing.h"
#include "multiplexer/LightRing.h"
#include "controllers/IRSensorController.h"
#include "controllers/LightSensorController.h"
#include "controllers/OffenseController.h"
#include "sensors/Gyro.h"
#include "motion/MotorClass.h"
#include "goal_detection/goal.h"

IRRing   hwIR;
LightCLuster hwLT;
IRSensorController   irCtrl(hwIR, 0.6f);
LightSensorController ltCtrl(hwLT, 0.6f);
Gyro gyro;
MotorClass drive;
OffenceController offence(irCtrl, ltCtrl, gyro, drive);

void setup(){
  Serial.begin(115200); while(!Serial){}
  Serial.println(F("Offense main online"));
  gyro.begin();
  offence.begin();
}

void loop(){
  offence.step();
  delay(10);
}
