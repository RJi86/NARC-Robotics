#include <math.h>
#include <Arduino.h>
#include <MotorClass.h>

const float pi = 3.14159265358979323846;

MotorClass::MotorClass() {
  for (byte i = 0; i < 4; i++) {
    pinMode(pinArray[i][0], OUTPUT);
    pinMode(pinArray[i][1], OUTPUT);
    pinMode(pinArray[i][2], OUTPUT);

    analogWrite(pinArray[i][1], 0);
  } // pins - direction (+,-), power, brake(0/1)
}

void MotorClass::MoveDirection(float theta, float speed) {
  const float value = sqrt(2) / 2; // sin(45)

  const float speedJust[4][2] = {
    { -value, value}, 
    { -value, -value}, 
    { value, -value}, 
    { value, values}
  };

  if (theta == pi/2){
    float x = 0;
    float y = 1;
  }
  else if (theta == 3*pi/2)
  {
    float x = 0;
    float y = -1;
  }
  

  float motion[3] = {0}; // four motor speeds
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

  for (byte i = 0; i < 2; i++) {
    if (motion[i] < 0) {
      digitalWrite(pinArray[i][0], HIGH);  // move backward if velocity is negative
      digitalWrite(pinArray[i][2], LOW);
    } else {
      digitalWrite(pinArray[i][0], LOW);
      digitalWrite(pinArray[i][2], HIGH);
    }
    if (motion[i+2] < 0){
      digitalWrite(pinArray[i+2][0], LOW);
      digitalWrite(pinArray[i+2][0], HIGH);
    } else {
      digitalWrite(pinArray[i+2][0], HIGH);
      digitalWrite(pinArray[i+2][0], LOW);
    }
    analogWrite(pinArray[i][1], abs(motion[i]));
    analogWrite(pinArray[i+2][1], abs(motion[i+2]));
  }

}

void MotorClass::Rotation(int countera, float speed) {
  switch(countera) {
    case 0: //motor clockwise, robot anticlockwise
      digitalWrite(pinArray[0][0], LOW);
      digitalWrite(pinArray[0][2], HIGH);
      analogWrite(pinArray[0][1], speed);
      digitalWrite(pinArray[1][0], LOW);
      digitalWrite(pinArray[1][2], HIGH);
      analogWrite(pinArray[1][1], speed);
      digitalWrite(pinArray[2][0], HIGH);
      digitalWrite(pinArray[2][2], LOW);
      analogWrite(pinArray[2][1], speed);
      digitalWrite(pinArray[3][0], HIGH);
      digitalWrite(pinArray[3][2], LOW);
      analogWrite(pinArray[3][1], speed);
    break;
    case 1: //motor anti, robot clockwise
      digitalWrite(pinArray[0][0], HIGH);
      digitalWrite(pinArray[0][2], LOW);
      analogWrite(pinArray[0][1], speed);
      digitalWrite(pinArray[1][0], HIGH);
      digitalWrite(pinArray[1][2], LOW);
      analogWrite(pinArray[1][1], speed);
      digitalWrite(pinArray[2][0], LOW);
      digitalWrite(pinArray[2][2], HIGH);
      analogWrite(pinArray[2][1], speed);
      digitalWrite(pinArray[3][0], LOW);
      digitalWrite(pinArray[3][2], HIGH);
      analogWrite(pinArray[3][1], speed);
    break;
  }
}

void MotorClass::Stop(){
  for (byte i = 0; i < 4; i++) {
    analogWrite(pinArray[i][1], 0);
  }
}