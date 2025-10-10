#include "MultiplexerAngle.h"
#include "GoalDetection.h"
#include "MotorClass.h"
#include "LDR.h"

// Create objects
MultiplexerAngle ballSensor;
GoalDetection goalDetector;
MotorClass motors;
LDR boundarySensor(42, 41, 40, A0); // Adjust pins as needed

// Robot states
enum RobotState {
  FIND_BALL,
  APPROACH_BALL,
  CATCH_BALL,
  FIND_GOAL,
  APPROACH_GOAL,
  AVOID_BOUNDARY
};

RobotState currentState = FIND_BALL;
RobotState previousState = FIND_BALL; // To return after boundary avoidance

// Settings
const float ROTATION_SPEED = 70;      // Speed for rotation (0-255)
const float APPROACH_SPEED = 120;     // Speed for moving toward ball/goal (0-255)
const float BOUNDARY_ESCAPE_SPEED = 150; // Speed to move away from boundary
const unsigned long BOUNDARY_ESCAPE_TIME = 500; // Time to move away from boundary (ms)
const unsigned long CATCH_TIME = 1000; // Time to confirm ball catch (ms)
const float ANGLE_STABLE_THRESHOLD = 3.0; // Max angle change to consider stable (degrees)

// Angle smoothing for distant balls
const int READINGS_COUNT = 5;
float recentReadings[READINGS_COUNT] = {-1, -1, -1, -1, -1};
int readingIndex = 0;

// Tracking variables
float lastBallAngle = -1;
unsigned long stableAngleStartTime = 0;
unsigned long boundaryDetectedTime = 0;
bool ballCaught = false;

void setup() {
  Serial.begin(115200);
  
  // Initialize systems
  ballSensor.begin();
  goalDetector.begin();
  boundarySensor.begin();
  
  Serial.println("Soccer Robot initialized!");
}

void loop() {
  // Update sensors
  ballSensor.findBallAngle(false, 100); // Less frequent debug output
  goalDetector.update();
  
  // Check for boundary first - highest priority
  float boundaryAngle = boundarySensor.angleOfLine();
  if (boundaryAngle >= 0 && currentState != AVOID_BOUNDARY) {
    Serial.print("Boundary detected at angle: ");
    Serial.println(boundaryAngle);
    previousState = currentState;
    currentState = AVOID_BOUNDARY;
    boundaryDetectedTime = millis();
  }
  
  float ballAngle = ballSensor.getLatestAngle();
  bool goalDetected = goalDetector.isEnemyGoalDetected();
  
  // Main state machine
  switch (currentState) {
    case AVOID_BOUNDARY:
      {
        // Calculate escape angle (opposite of boundary)
        float escapeAngle = boundaryAngle + PI;
        if (escapeAngle > 2 * PI) escapeAngle -= 2 * PI;
        
        // Move away from boundary
        motors.MoveDirection(escapeAngle, BOUNDARY_ESCAPE_SPEED);
        
        // After escaping for the set time, return to previous state
        if (millis() - boundaryDetectedTime > BOUNDARY_ESCAPE_TIME) {
          Serial.println("Boundary avoided, returning to previous state");
          currentState = previousState;
          motors.Stop();
        }
      }
      break;
      
    case FIND_BALL:
      if (ballAngle >= 0) {
        Serial.println("Ball found! Approaching...");
        currentState = APPROACH_BALL;
        
        // Reset angle smoothing
        for (int i = 0; i < READINGS_COUNT; i++) {
          recentReadings[i] = -1;
        }
      } else {
        // Rotate to search for ball
        motors.Rotation(0, ROTATION_SPEED);
      }
      break;
      
    case APPROACH_BALL:
      if (ballAngle < 0) {
        // Lost the ball
        Serial.println("Ball lost! Searching again...");
        currentState = FIND_BALL;
        motors.Stop();
      } else {
        // Get smoothed angle for more stable movement
        float smoothedAngle = getSmoothedAngle(ballAngle);
        
        // Calculate confidence based on sensor count
        int activeSensors = countActiveSensors();
        float confidence = float(activeSensors) / 10.0; // 0.1 to ~2.4
        if (confidence > 1.0) confidence = 1.0;
        
        // Adjust speed based on confidence (slower for distant balls)
        float adjustedSpeed = APPROACH_SPEED * (0.5 + (0.5 * confidence));
        
        // Convert angle to radians and approach ball
        float approachAngle = smoothedAngle * (PI / 180.0);
        
        // Move toward ball with adjusted speed
        motors.MoveDirection(approachAngle, adjustedSpeed);
        
        // If we have a very strong reading (ball is close), prepare to catch
        if (activeSensors >= 5 || (activeSensors >= 3 && abs(smoothedAngle) < 10.0)) {
          Serial.println("Ball close! Confirming catch...");
          currentState = CATCH_BALL;
          stableAngleStartTime = millis();
          lastBallAngle = smoothedAngle;
        }
      }
      break;
      
    case CATCH_BALL:
      if (ballAngle < 0) {
        // Lost the ball
        Serial.println("Ball lost during catch! Searching again...");
        currentState = FIND_BALL;
        motors.Stop();
        ballCaught = false;
      } else {
        // Get smoothed angle
        float smoothedAngle = getSmoothedAngle(ballAngle);
        
        // Slowly approach if not centered (mouth alignment)
        if (abs(smoothedAngle) > 15.0) {
          // Ball not centered, adjust position
          float approachAngle = smoothedAngle * (PI / 180.0);
          motors.MoveDirection(approachAngle, APPROACH_SPEED * 0.4);
          
          // Reset timer since we're still moving
          stableAngleStartTime = millis();
          lastBallAngle = smoothedAngle;
        } else {
          // Ball is centered, check stability
          motors.Stop();
          
          // Check if angle is stable (ball caught)
          if (abs(smoothedAngle - lastBallAngle) <= ANGLE_STABLE_THRESHOLD) {
 