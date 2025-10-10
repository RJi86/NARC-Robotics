#include "SoccIR/MultiplexerAngle.h"
#include "SoccVision/GoalDetection.h"
#include "SoccMotion/VectorMotorClass.h"
#include "SoccLDR/LDR.h"
#include "SoccMouth/Capture.h"
#include "SoccButton/Button.h"
#include <Arduino.h>

// Create objects
MultiplexerAngle ballSensor;
GoalDetection goalDetector;
VectorMotorControl motors;
LDR boundarySensor(42, 41, 40, A0); // Adjust pins as needed
BallCatcher ballCatcher(A1, 2.15);  // Analog pin for ball capture sensor
Button controlButton(6);            // Digital pin for control button

// Robot states
enum RobotState {
  STOPPED,
  FIND_BALL,
  APPROACH_BALL,
  ALIGN_FOR_CATCH,
  DRIBBLE_TOWARD_GOAL,
  AVOID_BOUNDARY
};

// State variables
RobotState currentState = STOPPED;
RobotState previousState = FIND_BALL; // State to return to after boundary avoidance or button press

// Settings
const float ROTATION_SPEED = 70;      // Speed for rotation (0-255)
const float APPROACH_SPEED = 120;     // Speed for moving toward ball (0-255)
const float DRIBBLE_SPEED = 150;      // Speed for moving with the ball (0-255)
const float BOUNDARY_ESCAPE_SPEED = 150; // Speed to move away from boundary
const unsigned long BOUNDARY_ESCAPE_TIME = 500; // Time to move away from boundary (ms)
const float BALL_ALIGN_THRESHOLD = 15.0; // Max degrees off to consider aligned with ball
const float GOAL_ALIGNMENT_WEIGHT = 0.3;  // How much to prioritize goal alignment (0-1)

// Angle smoothing for ball detection
const int READINGS_COUNT = 5;
float recentBallReadings[READINGS_COUNT] = {-1, -1, -1, -1, -1};
int readingIndex = 0;

// Timers and trackers
unsigned long boundaryDetectedTime = 0;
unsigned long ballAlignStartTime = 0;
bool ballAlignmentStable = false;
unsigned long lastStateChangeTime = 0;
unsigned long stateEnterTime = 0;
unsigned long debugOutputTime = 0;
bool buttonPressed = false;

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  
  // Initialize all systems
  ballSensor.begin();
  goalDetector.begin();
  goalDetector.enableDebugOutput(true);
  motors.begin();
  boundarySensor.begin();
  ballCatcher.begin();
  controlButton.begin();
  
  // Set initial state
  enterState(STOPPED);
  
  Serial.println("Vector-based Soccer Robot initialized!");
  Serial.println("Press button to start...");
}

void loop() {
  // Update button state first - this can interrupt any action
  if (controlButton.check()) {
    buttonPressed = true;
  }
  
  // Process button press
  if (buttonPressed) {
    if (currentState == STOPPED) {
      // Resume previous state or default to FIND_BALL
      enterState(previousState != STOPPED ? previousState : FIND_BALL);
    } else {
      // Remember current state and stop
      previousState = currentState;
      enterState(STOPPED);
    }
    buttonPressed = false;
  }
  
  // If stopped, don't process anything else
  if (currentState == STOPPED) {
    return;
  }
  
  // Update all sensors
  ballSensor.findBallAngle(false, 100);
  goalDetector.update();
  ballCatcher.update();
  
  // Check for boundary - highest priority
  float boundaryAngle = boundarySensor.angleOfLine();
  if (boundaryAngle >= 0 && currentState != AVOID_BOUNDARY) {
    Serial.print("Boundary detected at angle: ");
    Serial.println(boundaryAngle);
    previousState = currentState;
    enterState(AVOID_BOUNDARY);
    boundaryDetectedTime = millis();
  }
  
  // Get sensor data for state machine
  float rawBallAngle = ballSensor.getLatestAngle();
  float ballAngle = getSmoothedAngle(rawBallAngle);
  bool enemyGoalDetected = goalDetector.isEnemyGoalDetected();
  bool enemyGoalInFront = goalDetector.isEnemyGoalInFront();
  bool ballCaptured = ballCatcher.isBallCaptured();
  
  // Print debug info occasionally
  if (millis() - debugOutputTime > 1000) {
    printDebugInfo(ballAngle, enemyGoalDetected, enemyGoalInFront, ballCaptured);
    debugOutputTime = millis();
  }
  
  // Main state machine
  switch (currentState) {
    case AVOID_BOUNDARY:
      handleBoundaryAvoidance(boundaryAngle);
      break;
      
    case FIND_BALL:
      handleFindBall(ballAngle, enemyGoalDetected);
      break;
      
    case APPROACH_BALL:
      handleApproachBall(ballAngle, enemyGoalDetected, enemyGoalInFront, ballCaptured);
      break;
      
    case ALIGN_FOR_CATCH:
      handleAlignForCatch(ballAngle, enemyGoalDetected, enemyGoalInFront, ballCaptured);
      break;
      
    case DRIBBLE_TOWARD_GOAL:
      handleDribbleTowardGoal(ballCaptured, enemyGoalDetected, enemyGoalInFront);
      break;
      
    case STOPPED:
      // Nothing to do here, just keep motors stopped
      motors.Stop();
      break;
  }
  
  // Short delay to prevent CPU overload
  delay(10);
}

// State handlers
void handleBoundaryAvoidance(float boundaryAngle) {
  // Calculate escape angle (opposite of boundary)
  float escapeAngle = boundaryAngle + PI;
  if (escapeAngle > 2 * PI) escapeAngle -= 2 * PI;
  
  // Move away from boundary
  motors.MoveDirection(escapeAngle, BOUNDARY_ESCAPE_SPEED);
  
  // After escaping for the set time, return to previous state
  if (millis() - boundaryDetectedTime > BOUNDARY_ESCAPE_TIME) {
    Serial.println("Boundary avoided, returning to previous state");
    enterState(previousState);
  }
}

void handleFindBall(float ballAngle, bool enemyGoalDetected) {
  // If ball is detected, move to approach
  if (ballAngle >= 0) {
    Serial.println("Ball found! Approaching...");
    enterState(APPROACH_BALL);
    return;
  }
  
  // No ball detected, rotate to search
  // If enemy goal is detected, rotate in the direction that keeps it in view
  if (enemyGoalDetected) {
    // Check if goal is more in left or right side of view
    const GoalData& goal = goalDetector.getEnemyGoalData();
    // Rotate clockwise if goal is on left side (to keep it in view)
    bool rotateClockwise = (goal.x < 160); // Assuming camera center is around 160
    motors.Rotation(rotateClockwise ? 0 : 1, ROTATION_SPEED * 0.7);
  } else {
    // No goal in sight, just rotate clockwise
    motors.Rotation(0, ROTATION_SPEED);
  }
}

void handleApproachBall(float ballAngle, bool enemyGoalDetected, bool enemyGoalInFront, bool ballCaptured) {
  // If ball is lost, go back to finding the ball
  if (ballAngle < 0) {
    Serial.println("Ball lost! Searching again...");
    enterState(FIND_BALL);
    return;
  }
  
  // If ball is captured, move to dribble toward goal
  if (ballCaptured) {
    Serial.println("Ball captured! Moving toward goal...");
    enterState(DRIBBLE_TOWARD_GOAL);
    return;
  }
  
  // If ball is close and well-aligned, switch to alignment mode
  int activeSensors = countActiveSensors();
  if (activeSensors >= 3 && abs(ballAngle) < 30.0) {
    Serial.println("Ball close! Aligning for catch...");
    enterState(ALIGN_FOR_CATCH);
    return;
  }
  
  // Approach ball while trying to keep goal in front if possible
  approachBallKeepingGoalInView(ballAngle, enemyGoalDetected, enemyGoalInFront);
}

void handleAlignForCatch(float ballAngle, bool enemyGoalDetected, bool enemyGoalInFront, bool ballCaptured) {
  // If ball is lost, go back to finding the ball
  if (ballAngle < 0) {
    Serial.println("Ball lost during alignment! Searching again...");
    enterState(FIND_BALL);
    return;
  }
  
  // If ball is captured, move to dribble toward goal
  if (ballCaptured) {
    Serial.println("Ball captured! Moving toward goal...");
    enterState(DRIBBLE_TOWARD_GOAL);
    return;
  }
  
  // We need to position the robot so that:
  // 1. The ball is directly in front (angle near 0)
  // 2. The enemy goal is in the front camera view
  
  if (abs(ballAngle) <= BALL_ALIGN_THRESHOLD) {
    // We're well-aligned with the ball, move forward slowly
    float approachSpeed = APPROACH_SPEED * 0.5;
    motors.MoveDirection(0, approachSpeed);
    
    // Check if we need to adjust rotation to keep goal in front
    if (enemyGoalDetected && !enemyGoalInFront) {
      // Goal detected but in rear - need to adjust
      // Move slowly with slight rotation to keep goal in front view
      motors.MoveWithRotation(0, 0.2, approachSpeed * 0.5);
    }
  } else {
    // Not aligned with ball yet, move to align
    // Calculate alignment movement with small goal orientation bias
    alignForCatchWithGoalInView(ballAngle, enemyGoalDetected, enemyGoalInFront);
  }
}

void handleDribbleTowardGoal(bool ballCaptured, bool enemyGoalDetected, bool enemyGoalInFront) {
  // If we lose the ball, go back to finding the ball
  if (!ballCaptured) {
    Serial.println("Ball lost! Searching again...");
    enterState(FIND_BALL);
    return;
  }
  
  if (!enemyGoalDetected) {
    // Can't see goal, rotate to find it
    Serial.println("Goal not visible, rotating to find...");
    motors.Rotation(0, ROTATION_SPEED * 0.6);
    return;
  }
  
  // Move toward the enemy goal
  const GoalData& goal = goalDetector.getEnemyGoalData();
  
  // If goal is in rear view, we need to turn around
  if (!enemyGoalInFront) {
    // Rotate to face goal
    Serial.println("Goal in rear view, rotating to face...");
    motors.Rotation(0, ROTATION_SPEED * 0.7);
    return;
  }
  
  // Goal is in front view, move toward it
  float direction = 0.0;  // Forward
  float rotation = 0.0;   // No rotation initially
  
  // Calculate offset from center (assuming x position of 160 is center)
  float centerOffset = goal.x - 160;
  
  // Add rotation component based on how off-center the goal is
  if (abs(centerOffset) > 20) {
    rotation = -centerOffset / 320.0; // Normalize to range -0.5 to 0.5
    rotation = constrain(rotation, -0.3, 0.3); // Limit rotation
  }
  
  // Move toward goal with potential slight rotation
  motors.MoveWithRotation(direction, rotation, DRIBBLE_SPEED);
}

// Helper functions
void enterState(RobotState newState) {
  // Handle exiting current state
  switch (currentState) {
    case APPROACH_BALL:
    case ALIGN_FOR_CATCH:
    case DRIBBLE_TOWARD_GOAL:
    case AVOID_BOUNDARY:
      motors.Stop();
      break;
    default:
      break;
  }
  
  // Update state
  currentState = newState;
  stateEnterTime = millis();
  
  // Handle entering new state
  switch (newState) {
    case STOPPED:
      motors.Stop();
      Serial.println("State: STOPPED - Press button to resume");
      break;
    case FIND_BALL:
      Serial.println("State: FIND_BALL - Rotating to search for ball");
      break;
    case APPROACH_BALL:
      Serial.println("State: APPROACH_BALL - Moving toward ball");
      break;
    case ALIGN_FOR_CATCH:
      Serial.println("State: ALIGN_FOR_CATCH - Aligning to catch ball");
      ballAlignStartTime = millis();
      ballAlignmentStable = false;
      break;
    case DRIBBLE_TOWARD_GOAL:
      Serial.println("State: DRIBBLE_TOWARD_GOAL - Moving toward goal with ball");
      break;
    case AVOID_BOUNDARY:
      Serial.println("State: AVOID_BOUNDARY - Moving away from boundary");
      break;
  }
  
  lastStateChangeTime = millis();
}

void approachBallKeepingGoalInView(float ballAngle, bool enemyGoalDetected, bool enemyGoalInFront) {
  float approachAngle = ballAngle * (PI / 180.0);
  
  // Adjust speed based on active sensors (more active = closer ball = faster approach)
  int activeSensors = countActiveSensors();
  float confidence = constrain(float(activeSensors) / 8.0, 0.1, 1.0);
  float adjustedSpeed = APPROACH_SPEED * (0.5 + (0.5 * confidence));
  
  // If enemy goal is detected in rear view, we try to rotate while moving
  if (enemyGoalDetected && !enemyGoalInFront) {
    // Add a slight rotation component to get the goal into front view
    float rotationComponent = 0.2; // Small rotation to bring goal to front
    motors.MoveWithRotation(approachAngle, rotationComponent, adjustedSpeed * 0.7);
  } else {
    // Standard approach
    motors.MoveDirection(approachAngle, adjustedSpeed);
  }
}

void alignForCatchWithGoalInView(float ballAngle, bool enemyGoalDetected, bool enemyGoalInFront) {
  float approachAngle = ballAngle * (PI / 180.0);
  float rotationComponent = 0.0;
  
  // Slow approach speed for careful alignment
  float adjustedSpeed = APPROACH_SPEED * 0.4;
  
  // If enemy goal is detected in rear view, add rotation component
  if (enemyGoalDetected && !enemyGoalInFront) {
    rotationComponent = 0.15; // Small rotation to bring goal to front
    motors.MoveWithRotation(approachAngle, rotationComponent, adjustedSpeed * 0.7);
  } else {
    // Standard alignment movement
    motors.MoveDirection(approachAngle, adjustedSpeed);
  }
}

// Get smoothed angle by averaging recent readings
float getSmoothedAngle(float newAngle) {
  // Add new reading to array
  recentBallReadings[readingIndex] = newAngle;
  readingIndex = (readingIndex + 1) % READINGS_COUNT;
  
  // Count valid readings and calculate average
  float sum = 0;
  int validCount = 0;
  for (int i = 0; i < READINGS_COUNT; i++) {
    if (recentBallReadings[i] >= 0) {
      sum += recentBallReadings[i];
      validCount++;
    }
  }
  
  if (validCount == 0) return -1;
  return sum / validCount;
}

// Count how many IR sensors are active (for ball proximity detection)
int countActiveSensors() {
  int count = 0;
  for (int i = 0; i < 24; i++) {
    if (ballSensor.getSensorDetection(i)) {
      count++;
    }
  }
  return count;
}

// Print debug information
void printDebugInfo(float ballAngle, bool enemyGoalDetected, bool enemyGoalInFront, bool ballCaptured) {
  Serial.println("==== Robot Status ====");
  
  // Print state
  Serial.print("State: ");
  switch (currentState) {
    case STOPPED: Serial.println("STOPPED"); break;
    case FIND_BALL: Serial.println("FIND_BALL"); break;
    case APPROACH_BALL: Serial.println("APPROACH_BALL"); break;
    case ALIGN_FOR_CATCH: Serial.println("ALIGN_FOR_CATCH"); break;
    case DRIBBLE_TOWARD_GOAL: Serial.println("DRIBBLE_TOWARD_GOAL"); break;
    case AVOID_BOUNDARY: Serial.println("AVOID_BOUNDARY"); break;
  }
  
  // Print ball info
  Serial.print("Ball: ");
  if (ballAngle >= 0) {
    Serial.print("Detected at angle ");
    Serial.print(ballAngle, 1);
    Serial.print(" degrees, ");
    Serial.print(countActiveSensors());
    Serial.println(" active sensors");
  } else {
    Serial.println("Not detected");
  }
  
  // Print goal info
  Serial.print("Enemy Goal: ");
  if (enemyGoalDetected) {
    Serial.print("Detected (");
    Serial.print(enemyGoalInFront ? "FRONT" : "REAR");
    Serial.println(")");
  } else {
    Serial.println("Not detected");
  }
  
  // Print ball capture status
  Serial.print("Ball Captured: ");
  Serial.println(ballCaptured ? "YES" : "NO");
  
  Serial.println("====================");
}