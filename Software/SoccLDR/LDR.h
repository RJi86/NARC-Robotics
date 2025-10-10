#ifndef LDR_H
#define LDR_H

#include <Arduino.h>

typedef struct {
  bool A0Val;
  bool A1Val;
  bool A2Val;
  byte A0Pin;
  byte A1Pin;
  byte A2Pin;
  int outputPin;
} Multiplexer_LDR;

class LDR {
public:
    LDR(byte a0Pin, byte a1Pin, byte a2Pin, int outputPin);
    void begin();
    int readLightSensor(int sensorIndex);
    float angleOfLine();
    void readAllLightSensors();

private:
    Multiplexer_LDR multiplexer;
    const float SensorAngles[8]{
      22.5, 67.5, 112.5, 157.5, 202.5, 247.5, 292.5, 337.5
    };
    const float LDR_DIFFERENCE[8]{
      10.0, 10.0, 10.0, 10, 10, 10, 10, 5 // 8 variances
    };
    float green_reading[8] = {0,0,0,0,0,0,0,0}; // edit this on the day
    float averageAngleCircular(const bool detections[8]);

};

#endif // LDR_H