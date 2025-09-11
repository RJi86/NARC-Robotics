#include <math.h>
#include "../SoftwareSerialTX/src/SoftwareSerialTX.h"
#include <Arduino.h>
#include <MotorClass.h>

MotorClass::MotorClass() {
  for (byte i = 0; i < 4; i++) {
    pinMode(pinArray[i][0], OUTPUT);
    pinMode(pinArray[i][1], OUTPUT);
    pinMode(pinArray[i][2], OUTPUT);
    digitalWrite(pinArray[i][2], LOW);
  } // pins - direction (+,-), power, brake(0/1)
}

void MotorClass::MoveDirection(float theta, float speed) {
  const float value = sqrt(2) / 2; // sin(45)

  const float speedJust[4][3] = {
    { -value, value, 0.25 }, 
    { -value, -value, 0.25 }, 
    { value, -value, 0.25 }, 
    { value, value, 0.25 }
  };

  float motion[4] = {0}; // four motor speeds
  float x = 1;
  float y = tan(theta);
  //float w = 0;  // no rotation

  if (theta > _pi / 4) { // if angle > 45, tan > 1
    y = 1;
    x = 1 / tan(theta);
  }

  for (byte i = 0; i < 4; i++) {
    motion[i] = speedJust[i][0] * x + speedJust[i][1] * y; //+ speedJust[i][2] * w;
    motion[i] *= speed;  // Scale to intended speed
  }

  for (byte i = 0; i < 4; i++) {
    if (motion[i] < 0) {
      digitalWrite(pinArray[i][0], LOW);  // move backward if velocity is negative
    }
    analogWrite(pinArray[i][1], abs(motion[i]));
  }

}

