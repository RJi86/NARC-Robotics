#include <Wire.h>
#include <goal.h>

// Goal detection data
GoalData enemyGoal = {false, 0, 0, 0, 0};
GoalData homeGoal = {false, 0, 0, 0, 0};

// Constants
const unsigned long DETECTION_TIMEOUT = 500; // Consider detection lost after 500ms
const int ARDUINO_I2C_ADDRESS = 0x08;

void setup() {
  // Initialize I2C as slave
  Wire.begin(ARDUINO_I2C_ADDRESS);
  Wire.onReceive(receiveGoalData);
  
  // Initialize USB serial for debugging
  Serial.begin(9600);
  
  // Set up built-in LED for visual indication
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.println("Arduino ready to receive OpenMV data via I2C");
}

void loop() {
  // Check if detections are still valid (not timed out)
  checkDetectionTimeout();
  
  // Print current goal status
  printGoalStatus();
  
  // Visual indication when goals are detected
  digitalWrite(LED_BUILTIN, enemyGoal.detected || homeGoal.detected);
  
  delay(100);
}

void receiveGoalData(int numBytes) {
  if (numBytes != 7) {
    return; // Invalid data length
  }
  
  // Read data: [goal_type, distance_high, distance_low, x_high, x_low, y_high, y_low]
  char goalType = Wire.read();
  int distanceHigh = Wire.read();
  int distanceLow = Wire.read();
  int xHigh = Wire.read();
  int xLow = Wire.read();
  int yHigh = Wire.read();
  int yLow = Wire.read();
  
  // Reconstruct values
  float distance = ((distanceHigh << 8) | distanceLow) / 10.0; // Convert back to float
  int x = (xHigh << 8) | xLow;
  int y = (yHigh << 8) | yLow;
  
  unsigned long currentTime = millis();
  
  // Update appropriate goal data
  if (goalType == 'E') {
    enemyGoal.detected = true;
    enemyGoal.distance = distance;
    enemyGoal.x = x;
    enemyGoal.y = y;
    enemyGoal.lastUpdateTime = currentTime;
  } 
  else if (goalType == 'H') {
    homeGoal.detected = true;
    homeGoal.distance = distance;
    homeGoal.x = x;
    homeGoal.y = y;
    homeGoal.lastUpdateTime = currentTime;
  }
}

void checkDetectionTimeout() {
  unsigned long currentTime = millis();
  
  // Check if enemy goal detection has timed out
  if (enemyGoal.detected && (currentTime - enemyGoal.lastUpdateTime > DETECTION_TIMEOUT)) {
    enemyGoal.detected = false;
  }
  
  // Check if home goal detection has timed out
  if (homeGoal.detected && (currentTime - homeGoal.lastUpdateTime > DETECTION_TIMEOUT)) {
    homeGoal.detected = false;
  }
}

void printGoalStatus() {
  // Print enemy goal status
  Serial.print("Enemy Goal: ");
  if (enemyGoal.detected) {
    Serial.print("Detected at ");
    Serial.print(enemyGoal.distance);
    Serial.print("cm, Position: (");
    Serial.print(enemyGoal.x);
    Serial.print(",");
    Serial.print(enemyGoal.y);
    Serial.println(")");
  } else {
    Serial.println("Not detected");
  }
  
  // Print home goal status
  Serial.print("Home Goal: ");
  if (homeGoal.detected) {
    Serial.print("Detected at ");
    Serial.print(homeGoal.distance);
    Serial.print("cm, Position: (");
    Serial.print(homeGoal.x);
    Serial.print(",");
    Serial.print(homeGoal.y);
    Serial.println(")");
  } else {
    Serial.println("Not detected");
  }
}