#include <Wire.h>
#include <goal.h>

// I2C Configuration
#define I2C_ADDRESS 0x08
#define DATA_PACKET_SIZE 5

// Goal detection data
GoalData enemyGoal = {false, 0, 0, 0, 0};
GoalData homeGoal = {false, 0, 0, 0, 0};

// Constants
const unsigned long DETECTION_TIMEOUT = 500; // Consider detection lost after 500ms

void setup() {
  // Initialize I2C as slave
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
  
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

void receiveEvent(int numBytes) {
  if (numBytes == DATA_PACKET_SIZE) {
    // Read the data packet
    char goalType = Wire.read();
    uint8_t distanceHigh = Wire.read();
    uint8_t distanceLow = Wire.read();
    uint8_t xPos = Wire.read();
    uint8_t yPos = Wire.read();
    
    // Reconstruct distance from two bytes
    float distance = (float)((distanceHigh << 8) | distanceLow);
    
    // Update the appropriate goal data
    if (goalType == 'E') {
      enemyGoal.detected = true;
      enemyGoal.distance = distance;
      enemyGoal.x = (int)xPos;
      enemyGoal.y = (int)yPos;
      enemyGoal.lastUpdateTime = millis();
    } 
    else if (goalType == 'H') {
      homeGoal.detected = true;
      homeGoal.distance = distance;
      homeGoal.x = (int)xPos;
      homeGoal.y = (int)yPos;
      homeGoal.lastUpdateTime = millis();
    }
  }
  
  // Clear any remaining bytes
  while (Wire.available()) {
    Wire.read();
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