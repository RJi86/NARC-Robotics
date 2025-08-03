#ifndef OmniDrive_h
#define OmniDrive_h
#include <Arduino.h>

class OmniDrive {
  public:
    OmniDrive(byte brakePin, byte dirPin, byte pwmPin);
    void OmniGo(int speed);
  private:
    byte _brakePin, _dirPin, _pwmPin;
};

#endif