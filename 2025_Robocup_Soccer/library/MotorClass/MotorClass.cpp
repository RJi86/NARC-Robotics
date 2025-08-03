#include <math.h>
#include <SoftwareSerial.h>
#include <Arduino.h>
#include <MotorClass.h>

MotorClass::MotorClass() {
  for (byte i = 0; i < 4; i++) {
    pinMode(pinArray[i][0], OUTPUT);
    pinMode(pinArray[i][1], OUTPUT);
    pinMode(pinArray[i][2], OUTPUT);
    digitalWrite(pinArray[i][2], LOW);
  }
}

void MotorClass::MoveDirection(float theta) {
  const float value = sqrt(2) / 4;

  const float speedJust[4][3] = {
    { -value, value, 0.25 }, 
    { -value, -value, 0.25 }, 
    { value, -value, 0.25 }, 
    { value, value, 0.25 }
  };

  float motion[4] = {0};
  float x = 1;
  float y = tan(theta);
  //float w = 0;  // no rotation

  if (theta > _pi / 4) {
    y = 1;
    x = 1 / tan(theta);
  }

  for (byte i = 0; i < 4; i++) {
    motion[i] = speedJust[i][0] * x + speedJust[i][1] * y; //+ speedJust[i][2] * w;
    motion[i] *= 255;  // Scale to Pin range (-255 to 255)
  }

  for (byte i = 0; i < 4; i++) {
    if (motion[i] < 0) {
      digitalWrite(pinArray[i][0], LOW);  // move backward if velocity is negative
    }
    analogWrite(pinArray[i][1], abs(motion[i]));
  }
}