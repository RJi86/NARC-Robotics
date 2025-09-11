#include <math.h>
#include "../SoftwareSerialTX/src/SoftwareSerialTX.h"
#include "motors/MotorClass.h"
#include <Arduino.h>

MotorClass omnimotion();


void setup() {

}

void loop(){
  omnimotion.MoveDirection(pi/4); // commanding the robot to move at certain angle
}
