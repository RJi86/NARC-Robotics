#include <math.h>
#include <SoftwareSerial.h>
#include <MotorClass.h>

const float pi = 3.14159265358979323846;

int pins[4][3] = {
  {4, 3, 5},
  {8, 7, 9},
  {12, 11, 13},
  {16, 15, 17}
}; // pin numbers

MotorClass omnimotion(pins);


// const int dirPin0 = 4;
// const int pwmPin0 = 3;
// const int brakePin0 = 5;


void setup() {

}

void loop(){
  omnimotion.MoveDirection(pi/4); // commanding the robot to move at certain angle
}