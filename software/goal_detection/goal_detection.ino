// Robot_Control.ino - Main robot control program
#include <goal.h>

// Goal detection data
GoalData enemyGoal = {false, 0, 0, 0, 0, 0};
GoalData homeGoal = {false, 0, 0, 0, 0, 0};

// Constants
const unsigned long DETECTION_TIMEOUT = 500; // Consider detection lost after 500ms

void setup() {
  // Initialize serial communication with OpenMV camera
  Serial1.begin(9600);  // Use Serial1 for OpenMV
  
  // Initialize USB serial for debugging
  Serial.begin(9600);
  
  // Set up built-in LED for visual indication
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.println("Arduino ready to receive OpenMV data");
}

void loop() {
  // Read vision data from OpenMV camera
  readVisionData();
  
  // Check if detections are still valid (not timed out)
  checkDetectionTimeout();
  
  // Print current goal status
  printGoalStatus();
  
  // Visual indication when goals are detected
  digitalWrite(LED_BUILTIN, enemyGoal.detected || homeGoal.detected);
  
  delay(100); // Short delay
}

void readVisionData() {
  if (Serial1.available()) {  // Use Serial1 to read from OpenMV
    String data = Serial1.readStringUntil('\n');
    parseVisionData(data);
  }
}

void parseVisionData(String data) {
  // Format expected: X,distance,width,x,y
  // Where X is 'E' for enemy goal or 'H' for home goal
  
  // Check if data is valid
  if (data.length() < 3) return;
  
  // Get the first character to determine type
  char type = data.charAt(0);
  
  // Get the rest of the data after the type
  int commaIndex = data.indexOf(',');
  if (commaIndex < 0) return;
  
  // Parse the data
  int startIndex = commaIndex + 1;
  
  // Parse distance
  commaIndex = data.indexOf(',', startIndex);
  if (commaIndex < 0) return;
  float distance = data.substring(startIndex, commaIndex).toFloat();
  startIndex = commaIndex + 1;
  
  // Parse width
  commaIndex = data.indexOf(',', startIndex);
  if (commaIndex < 0) return;
  float width = data.substring(startIndex, commaIndex).toFloat();
  startIndex = commaIndex + 1;
  
  // Parse x position
  commaIndex = data.indexOf(',', startIndex);
  if (commaIndex < 0) return;
  int x = data.substring(startIndex, commaIndex).toInt();
  startIndex = commaIndex + 1;
  
  // Parse y position
  int y = data.substring(startIndex).toInt();
  
  // Update the appropriate goal data
  if (type == 'E') {
    enemyGoal.detected = true;
    enemyGoal.distance = distance;
    enemyGoal.width = width;
    enemyGoal.x = x;
    enemyGoal.y = y;
    enemyGoal.lastUpdateTime = millis();
  } 
  else if (type == 'H') {
    homeGoal.detected = true;
    homeGoal.distance = distance;
    homeGoal.width = width;
    homeGoal.x = x;
    homeGoal.y = y;
    homeGoal.lastUpdateTime = millis();
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
    Serial.print("cm, Width: ");
    Serial.print(enemyGoal.width);
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
    Serial.print("cm, Width: ");
    Serial.print(homeGoal.width);
    Serial.print("cm, Position: (");
    Serial.print(homeGoal.x);
    Serial.print(",");
    Serial.print(homeGoal.y);
    Serial.println(")");
  } else {
    Serial.println("Not detected");
  }
}

// Add your other robot control functions here
// void setupMotors() { ... }
// void aimTowardsGoal(GoalData goal) { ... }
// etc.