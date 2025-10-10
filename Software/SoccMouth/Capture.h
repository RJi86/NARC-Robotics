#ifndef CAPTURE_H
#define CAPTURE_H

class BallCatcher {
  private:
    int sensorPin;
    float average;
    float threshold;
    bool ballCaptured;
    
  public:
    // Constructor
    BallCatcher(int pin, float thresholdMultiplier = 2.15);
    
    // Initialize and calibrate the sensor
    void begin();
    
    // Check if ball is captured
    void update();
    
    // Get current status
    bool isBallCaptured();
    
    // Reset the captured state
    void reset();
};

#endif