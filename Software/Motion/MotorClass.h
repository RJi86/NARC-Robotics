#ifndef MotorClass_h
#define MotorClass_h

#include <Arduino.h>

class MotorClass {
public:
  MotorClass();
  void MoveDirection(float theta, float speed);
  void Rotation(int countera, float speed);
  void Stop();
private:
  const float _pi = 3.14159265358979323846;

  struct {
    byte dirPin = 34;
    byte pwmPin = 9;
    byte brakePin = 35;
  } TR_Motor; // Motor 4

  struct {
    byte dirPin = 32;
    byte pwmPin = 8;
    byte brakePin = 33;
  } TL_Motor; // Motor 3

  struct {
    byte dirPin = 30;
    byte pwmPin = 7;
    byte brakePin = 31;
  } BL_Motor; // Motor 2

  struct {
    byte dirPin = 28;
    byte pwmPin = 6;
    byte brakePin = 29;
  } BR_Motor; // Motor 1

  const byte pinArray[4][3] = {
    { TR_Motor.dirPin, TR_Motor.pwmPin, TR_Motor.brakePin },
    { TL_Motor.dirPin, TL_Motor.pwmPin, TL_Motor.brakePin },
    { BL_Motor.dirPin, BL_Motor.pwmPin, BL_Motor.brakePin },
    { BR_Motor.dirPin, BR_Motor.pwmPin, BR_Motor.brakePin }
  };
};


#endif