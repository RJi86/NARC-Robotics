#ifndef MULTIPLEXER_ANGLE_H
#define MULTIPLEXER_ANGLE_H

#include "IRFilters.h"
#include <Arduino.h>

// Structure for multiplexer configuration
typedef struct {
  bool A0Val;
  bool A1Val;
  bool A2Val;
  byte A0Pin;
  byte A1Pin;
  byte A2Pin;
  int outputPin;
} Multiplexer;

class MultiplexerAngle {
public:
  // Constructor and initialization
  MultiplexerAngle();
  void begin();
  
  // Main function to find the ball angle
  void findBallAngle(bool dumpSensors = true, uint16_t period_ms = 500);
  
  // Gets the latest calculated ball angle (-1 if no ball detected)
  float getLatestAngle() const;
  
  // Access to sensor readings
  bool getSensorDetection(int sensorIndex) const;

private:
  // Define the angle for each sensor (in degrees)
  // Assuming sensors are arranged in a circle with sensor 0 at 0 degrees,
  // sensor 1 at 15 degrees, and so on
  static const int SENSOR_ANGLES[24];
  
  // Digital filters for sensor readings
  DigitalFilter filters[24]; //one per sensor
  
  // Multiplexer configurations
  Multiplexer multiplexer1;
  Multiplexer multiplexer2;
  Multiplexer multiplexer3;
  
  // Current sensor detections
  bool sensorDetections[24];
  
  // Latest calculated angle
  float latestAngle;
  
  // Timestamp for periodic output
  uint32_t lastOutputTime;
  
  // Helper methods
  bool readSensor(Multiplexer &mux, int sensorIndex);
  static inline float toRad(float deg) { return deg * 0.017453292519943295f; }
  static inline float toDeg(float rad) { return rad * 57.29577951308232f; }
  float averageAngleCircular(const bool detections[24]) const;
};

#endif // MULTIPLEXER_ANGLE_H