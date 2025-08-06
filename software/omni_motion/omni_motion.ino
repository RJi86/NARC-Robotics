#include <math.h>
#include <SoftwareSerial.h>
#include <MotorClass.h>

MotorClass omnimotion();


void setup() {

}

void loop(){
  omnimotion.MoveDirection(pi/4); // commanding the robot to move at certain angle
}
