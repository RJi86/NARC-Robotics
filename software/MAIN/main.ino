#include "MotorClass.h"

// IR Ball Detection Definitions
typedef struct {
  bool A0Val;
  bool A1Val;
  bool A2Val;
  byte A0Pin;
  byte A1Pin;
  byte A2Pin;
  int outputPin;
} Multiplexer;

// Define the angle for each sensor (in degrees)
// Assuming sensors are arranged in a circle with sensor 0 at 0 degrees,
// sensor 1 at 15 degrees, and so on
const int SENSOR_ANGLES[24] = {
  0, 15, 30, 45, 60, 75, 90, 105,         // Multiplexer 1 sensors (0-7)
  120, 135, 150, 165, 180, 195, 210, 225, // Multiplexer 2 sensors (8-15)
  240, 255, 270, 285, 300, 315, 330, 345  // Multiplexer 3 sensors (16-23)
};

// Define front sensors that would detect a ball in the "mouth"
const int FRONT_SENSORS[] = {0, 1, 23}; // 0°, 15°, and 345° sensors

Multiplexer multiplexer1;
Multiplexer multiplexer2;
Multiplexer multiplexer3;

// Create a motor controller instance
MotorClass motors;

// Robot state definitions
enum RobotState {
  IDLE,       // Waiting for ball detection
  ALIGNING,   // Rotating to face the ball
  APPROACHING,// Moving straight to catch the ball
  CAPTURED    // Ball is caught
};

// Movement parameters
const float DEFAULT_SPEED = 255.0;      // Default motor speed (0-255)
const float ROTATION_SPEED = 150.0;     // Speed for rotation
const float APPROACH_SPEED = 200.0;     // Speed for approaching the ball
const int NO_BALL_TIMEOUT = 2500;       // How long to keep moving without seeing ball (ms)
const int MIN_UPDATE_INTERVAL = 20;     // Minimum time between angle updates (ms)
const int ANGLE_TOLERANCE = 10;         // Acceptable angle deviation (degrees)
const int CONSISTENT_ANGLE_TIME = 500;  // Time ball must stay at consistent angle to consider captured (ms)
const int FRONT_DETECTION_THRESHOLD = 2; // Number of front sensors that must detect ball

// Program state variables
RobotState currentState = IDLE;         // Current state of the robot
int ballAngle = -1;                     // Current ball angle
int lastBallAngle = -1;                 // Last known ball angle
unsigned long lastBallDetection = 0;    // Last time ball was detected
unsigned long lastAngleUpdate = 0;      // Last time movement direction was updated
unsigned long consistentAngleStart = 0; // When ball angle became consistent
bool anglePreviouslyConsistent = false; // Whether angle was previously consistent

void setup() {
  Serial.begin(115200);
  
  // Set up multiplexer 1
  multiplexer1.A0Pin = 20;
  multiplexer1.A1Pin = 21;
  multiplexer1.A2Pin = 22;
  multiplexer1.outputPin = 2;
  
  // Set up multiplexer 2
  multiplexer2.A0Pin = 23;
  multiplexer2.A1Pin = 26;
  multiplexer2.A2Pin = 27;
  multiplexer2.outputPin = 3;
  
  // Set up multiplexer 3
  multiplexer3.A0Pin = 14;
  multiplexer3.A1Pin = 15;
  multiplexer3.A2Pin = 41;
  multiplexer3.outputPin = 4;
  
  // Set all multiplexer pins as OUTPUT
  pinMode(multiplexer1.A0Pin, OUTPUT);
  pinMode(multiplexer1.A1Pin, OUTPUT);
  pinMode(multiplexer1.A2Pin, OUTPUT);
  
  pinMode(multiplexer2.A0Pin, OUTPUT);
  pinMode(multiplexer2.A1Pin, OUTPUT);
  pinMode(multiplexer2.A2Pin, OUTPUT);
  
  pinMode(multiplexer3.A0Pin, OUTPUT);
  pinMode(multiplexer3.A1Pin, OUTPUT);
  pinMode(multiplexer3.A2Pin, OUTPUT);
  
  // Set all output pins as INPUT
  pinMode(multiplexer1.outputPin, INPUT);
  pinMode(multiplexer2.outputPin, INPUT);
  pinMode(multiplexer3.outputPin, INPUT);
  
  // Ensure motors are stopped at start
  motors.Stop();
  
  Serial.println("IR Ball Tracker Initialized");
  Serial.println("State: IDLE - Waiting for ball detection");
  delay(1000);
}

// Function to read a specific sensor from a multiplexer
bool readSensor(Multiplexer &mux, int sensorIndex) {
  // Set address pins based on sensor index (0-7)
  mux.A0Val = (sensorIndex & 0x01) > 0;  // Bit 0
  mux.A1Val = (sensorIndex & 0x02) > 0;  // Bit 1
  mux.A2Val = (sensorIndex & 0x04) > 0;  // Bit 2
  
  // Set the address pins
  digitalWrite(mux.A0Pin, mux.A0Val);
  digitalWrite(mux.A1Pin, mux.A1Val);
  digitalWrite(mux.A2Pin, mux.A2Val);
  
  // Read the output (0 = ball detected, 1 = no detection)
  return digitalRead(mux.outputPin) == 0;  // Return true if ball is detected
}

// Determines if the ball is detected by the front sensors (in the "mouth")
bool isBallInFront() {
  int frontDetections = 0;
  
  // Count how many front sensors detect the ball
  for (int i = 0; i < sizeof(FRONT_SENSORS) / sizeof(FRONT_SENSORS[0]); i++) {
    int sensorIndex = FRONT_SENSORS[i];
    
    // Determine which multiplexer to use based on sensor index
    if (sensorIndex < 8) {
      if (readSensor(multiplexer1, sensorIndex)) frontDetections++;
    } else if (sensorIndex < 16) {
      if (readSensor(multiplexer2, sensorIndex - 8)) frontDetections++;
    } else {
      if (readSensor(multiplexer3, sensorIndex - 16)) frontDetections++;
    }
  }
  
  return frontDetections >= FRONT_DETECTION_THRESHOLD;
}

int findBallAngle() {
  bool sensorDetections[24] = {false};  // Store which sensors detect the ball
  int detectionCount = 0;               // Count how many sensors detect the ball
  
  // Read all sensors from multiplexer 1
  for (int i = 0; i < 8; i++) {
    sensorDetections[i] = readSensor(multiplexer1, i);
    if (sensorDetections[i]) detectionCount++;
  }
  
  // Read all sensors from multiplexer 2
  for (int i = 0; i < 8; i++) {
    sensorDetections[i + 8] = readSensor(multiplexer2, i);
    if (sensorDetections[i + 8]) detectionCount++;
  }
  
  // Read all sensors from multiplexer 3
  for (int i = 0; i < 8; i++) {
    sensorDetections[i + 16] = readSensor(multiplexer3, i);
    if (sensorDetections[i + 16]) detectionCount++;
  }
  
  // If no ball is detected, return -1
  if (detectionCount == 0) {
    return -1;
  }
  
  // Calculate the average angle of all sensors that detect the ball
  int totalAngle = 0;
  for (int i = 0; i < 24; i++) {
    if (sensorDetections[i]) {
      totalAngle += SENSOR_ANGLES[i];
    }
  }
  
  return totalAngle / detectionCount;  // Return the average angle
}

// Function to check if the angle is within the tolerance of 0 degrees
bool isAngleAligned(int angle) {
  // Consider 0 and 360 to be the same
  if (angle > 360 - ANGLE_TOLERANCE) {
    angle = angle - 360;
  }
  
  return abs(angle) <= ANGLE_TOLERANCE;
}

// Function to determine the rotation direction to align with ball
// CORRECTED: Matches MotorClass.Rotation parameter meaning
// Returns 1 for clockwise, 0 for counterclockwise
int getRotationDirection(int angle) {
  // If angle is between 0 and 180, rotate clockwise (1)
  // If angle is between 180 and 360, rotate counterclockwise (0)
  return (angle > 0 && angle <= 180) ? 1 : 0;
}

void loop() {
  // Find the ball angle
  ballAngle = findBallAngle();
  unsigned long currentTime = millis();
  
  // Ball is detected
  if (ballAngle != -1) {
    lastBallDetection = currentTime;
    
    // State machine logic
    switch (currentState) {
      case IDLE:
        // Ball found, move to ALIGNING state
        currentState = ALIGNING;
        Serial.println("Ball detected! State: ALIGNING");
        break;
        
      case ALIGNING:
        // Check if the robot is aligned with the ball (ball is at 0 degrees ±tolerance)
        if (isAngleAligned(ballAngle)) {
          // Aligned with ball, move to APPROACHING state
          motors.Stop();
          currentState = APPROACHING;
          Serial.println("Aligned with ball. State: APPROACHING");
        } else {
          // Not aligned, keep rotating
          int rotationDir = getRotationDirection(ballAngle);
          motors.Rotation(rotationDir, ROTATION_SPEED);
          Serial.print("Rotating to align with ball at angle: ");
          Serial.print(ballAngle);
          Serial.print(" degrees. Direction: ");
          Serial.println(rotationDir == 1 ? "clockwise" : "counterclockwise");
        }
        break;
        
      case APPROACHING:
        // Moving straight towards ball
        if (isAngleAligned(ballAngle)) {
          // Still aligned, keep moving forward
          // Using direction 0 radians (0 degrees) to move forward
          motors.MoveDirection(0, APPROACH_SPEED);
          
          // Check for consistent angle (ball might be captured)
          if (!anglePreviouslyConsistent) {
            consistentAngleStart = currentTime;
            anglePreviouslyConsistent = true;
          } else if (currentTime - consistentAngleStart >= CONSISTENT_ANGLE_TIME && isBallInFront()) {
            // Ball has been at consistent angle and detected by front sensors
            currentState = CAPTURED;
            Serial.println("Ball captured! State: CAPTURED");
          }
        } else {
          // Lost alignment, go back to aligning
          anglePreviouslyConsistent = false;
          currentState = ALIGNING;
          Serial.println("Lost alignment, returning to ALIGNING state");
        }
        break;
        
      case CAPTURED:
        // Ball is captured, keep moving forward with it
        motors.MoveDirection(0, APPROACH_SPEED);
        
        // If ball is no longer in front, it might have been lost
        if (!isBallInFront()) {
          Serial.println("Ball may have been lost. Returning to IDLE state");
          motors.Stop();
          currentState = IDLE;
          Serial.println("State: IDLE - Waiting for ball detection");
        }
        break;
    }
    
    lastBallAngle = ballAngle;
  }
  // No ball detected
  else {
    // If we haven't seen the ball for a while, stop moving and return to IDLE
    if (currentTime - lastBallDetection > NO_BALL_TIMEOUT) {
      if (currentState != IDLE) {
        motors.Stop();
        currentState = IDLE;
        Serial.println("Ball lost - stopping motors");
        Serial.println("State: IDLE - Waiting for ball detection");
      }
    }
    
    anglePreviouslyConsistent = false;
  }
  
  // Small delay to avoid overwhelming the system
  delay(10);
}