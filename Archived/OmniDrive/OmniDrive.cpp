#include "OmniDrive.h"
#include <Arduino.h>

OmniDrive::OmniDrive(byte brakePin, byte dirPin, byte pwmPin) {
  pinMode(brakePin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPIN, OUTPUT);
  _brakePin = brakePin;
  _dirPin = dirPin;
  _pwmPin = pwmPin;
  digitalWrite(brakePin, LOW);
}

void OmniDrive::OmniGo(int speed) {
  if (speed > 0) {
    digitalWrite(_dirPin, HIGH);
    analogWrite(_pwmPin, abs(speed));
  } else if (speed < 0) {
    digitalWrite(_dirPin, LOW);
    analogWrite(_pwmPin, abs(speed));
  } else {
    digitalWrite(_brakePin, HIGH);
  }
}