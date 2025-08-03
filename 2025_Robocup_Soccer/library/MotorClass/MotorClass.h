#ifndef MotorClass_h
#define MotorClass_h

#include <Arduino.h>

class MotorClass {
public:
  MotorClass();
  void MoveDirection(float theta);
private:
  const float _pi = 3.14159265358979323846;
  struct {
    byte dirPin = 2;
    byte pwmPin = 3;
    byte brakePin = 4;
  } TL_Motor;

  struct {
    byte dirPin = 5;
    byte pwmPin = 6;
    byte brakePin = 7;
  } TR_Motor;

  struct {
    byte dirPin = 8;
    byte pwmPin = 9;
    byte brakePin = 10;
  } BL_Motor;

  struct {
    byte dirPin = 11;
    byte pwmPin = 12;
    byte brakePin = 13;
  } BR_Motor;

  const byte pinArray[4][3] = {
    { TL_Motor.dirPin, TL_Motor.pwmPin, TL_Motor.brakePin },
    { TR_Motor.dirPin, TR_Motor.pwmPin, TR_Motor.brakePin },
    { BL_Motor.dirPin, BL_Motor.pwmPin, BL_Motor.brakePin },
    { BR_Motor.dirPin, BR_Motor.pwmPin, BR_Motor.brakePin }
  };
};

#endif