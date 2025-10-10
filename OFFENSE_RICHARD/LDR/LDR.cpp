#include "LDR.h"
#include <Arduino.h>
#include <math.h>

namespace {
  inline float toRad(float deg) { return deg * 0.017453292519943295f; }
  inline float toDeg(float rad) { return rad * 57.29577951308232f; }

  // Average a few reads to build a stable baseline per channel.
  int baselineRead(Multiplexer &mux, int sensorIndex,
                   uint8_t SAMPLES = 8, uint16_t SETTLE_US = 200) {
    long acc = 0;
    for (uint8_t k = 0; k < SAMPLES; ++k) {
      // Address bits
      mux.A0Val = (sensorIndex & 0x01) > 0;
      mux.A1Val = (sensorIndex & 0x02) > 0;
      mux.A2Val = (sensorIndex & 0x04) > 0;

      digitalWrite(mux.A0Pin, mux.A0Val);
      digitalWrite(mux.A1Pin, mux.A1Val);
      digitalWrite(mux.A2Pin, mux.A2Val);

      delayMicroseconds(50);
      (void)analogRead(mux.outputPin); // throw-away read
      delayMicroseconds(SETTLE_US);
      acc += analogRead(mux.outputPin);
    }
    return (int)(acc / (long)SAMPLES);
  }
}

// ----- class implementation -----
LDR::LDR(byte a0Pin, byte a1Pin, byte a2Pin, int outputPin) {
  multiplexer.A0Pin     = a0Pin;
  multiplexer.A1Pin     = a1Pin;
  multiplexer.A2Pin     = a2Pin;
  multiplexer.outputPin = outputPin;

  multiplexer.A0Val = false;
  multiplexer.A1Val = false;
  multiplexer.A2Val = false;
}

void LDR::begin() {
  pinMode(multiplexer.A0Pin, OUTPUT);
  pinMode(multiplexer.A1Pin, OUTPUT);
  pinMode(multiplexer.A2Pin, OUTPUT);
  pinMode(multiplexer.outputPin, INPUT);

  // Teensy: no-op on boards that don't support it
  analogReadResolution(12);

  // Build baseline (green_reading[]) for each channel
  for (int i = 0; i < 8; ++i) {
    green_reading[i] = (float)baselineRead(multiplexer, i);
  }
}

int LDR::readLightSensor(int sensorIndex) {
  if (sensorIndex < 0 || sensorIndex > 7) return 0;

  multiplexer.A0Val = (sensorIndex & 0x01) > 0;
  multiplexer.A1Val = (sensorIndex & 0x02) > 0;
  multiplexer.A2Val = (sensorIndex & 0x04) > 0;

  digitalWrite(multiplexer.A0Pin, multiplexer.A0Val);
  digitalWrite(multiplexer.A1Pin, multiplexer.A1Val);
  digitalWrite(multiplexer.A2Pin, multiplexer.A2Val);

  delayMicroseconds(50);
  (void)analogRead(multiplexer.outputPin); // flush prior channel
  delayMicroseconds(200);

  return analogRead(multiplexer.outputPin);
}

float LDR::angleOfLine() {
  bool sensorDetections[8] = {false, false, false, false, false, false, false, false};

  for (int i = 0; i < 8; ++i) {
    int raw = readLightSensor(i);
    float diff = float(raw) - green_reading[i];
    if (diff > LDR_DIFFERENCE[i]) {
      sensorDetections[i] = true;
    }
  }

  const float avg = averageAngleCircular(sensorDetections);
  if (isnan(avg)) return -1.0f;   // no detections
  return avg;                     // [0,360)
}

void LDR::readAllLightSensors() {
  Serial.println("Light Sensor Readings:");
  for (int i = 0; i < 8; ++i) {
    int v = readLightSensor(i);
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(v);
  }
  Serial.println();
}

float LDR::averageAngleCircular(const bool detections[8]) {
  float sumX = 0.0f, sumY = 0.0f;
  int count = 0;
  for (int i = 0; i < 8; ++i) {
    if (!detections[i]) continue;
    const float th = toRad(SensorAngles[i]);  // <-- use SensorAngles
    sumX += cosf(th);
    sumY += sinf(th);
    count++;
  }
  if (count == 0) return NAN;

  const float EPS = 1e-4f;
  if (fabsf(sumX) < EPS && fabsf(sumY) < EPS) return NAN;


  float ang = atan2f(sumY, sumX);
  if (ang < 0.0f) ang += 2*PI;
  return ang;
}