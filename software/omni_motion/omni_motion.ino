#include <math.h>
#include "SoftwareSerialTX.h"
#include <"MotorClass.h">
#include <Arduino.h>

MotorClass omnimotion();


void setup() {

}

void loop(){
  omnimotion.MoveDirection(pi/4); // commanding the robot to move at certain angle
}
