#include "Capture.h"
#include <Arduino.h>

BallCatcher::BallCatcher(int pin, float thresholdMultiplier) {
  sensorPin = pin;
  threshold = thresholdMultiplier;
  average = 0.0;
  ballCaptured = false;
}

void BallCatcher::begin() {
  pinMode(sensorPin, INPUT_PULLUP);
  
  // Take initial readings to calculate average
  float total = 0.0;
  analogRead(sensorPin); // First reading is often inaccurate
  
  for (int i = 0; i < 5; i++) {
    total += float(analogRead(sensorPin));
    delay(20);
  }
  
  average = total / 5;
  Serial.print("Sensor calibrated. Average reading: ");
  Serial.println(average);
}

void BallCatcher::update() {
  float captureReading = float(analogRead(sensorPin));
  
  if (captureReading > threshold * average) {
    ballCaptured = true;
  }
}

bool BallCatcher::isBallCaptured() {
  return ballCaptured;
}

void BallCatcher::reset() {
  ballCaptured = false;
}