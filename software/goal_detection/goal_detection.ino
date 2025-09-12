// Robot_Control.ino - Main robot control program
#include <goal.h>

// Goal detection data
GoalData enemyGoal = {false, 0, 0, 0, 0, 0};
GoalData homeGoal = {false, 0, 0, 0, 0, 0};

// Constants
const unsigned long DETECTION_TIMEOUT = 1000; // Consider detection lost after 1 second
const unsigned long HEARTBEAT_INTERVAL = 2000; // Print status every 2 seconds

// Variables for heartbeat
unsigned long lastHeartbeat = 0;

void setup() {
  // Initialize serial communication with OpenMV camera
  Serial1.begin(9600);  // Use Serial1 for OpenMV
  
  // Initialize USB serial for debugging
  Serial.begin(9600);
  
  // Set up built-in LED for visual indication
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.println("Arduino ready to receive OpenMV data");
  Serial.println("Waiting for camera initialization...");
  
  // Clear any initial garbage from serial buffer
  while(Serial1.available()) {
    Serial1.read();
  }
}

void loop() {
  // Read vision data from OpenMV camera
  readVisionData();
  
  // Check if detections are still valid (not timed out)
  checkDetectionTimeout();
  
  // Print current goal status periodically
  unsigned long currentTime = millis();
  if (currentTime - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    printGoalStatus();
    lastHeartbeat = currentTime;
  }
  
  // Visual indication when goals are detected
  digitalWrite(LED_BUILTIN, enemyGoal.detected || homeGoal.detected);
  
  delay(50); // Reduced delay for better responsiveness
}

void readVisionData() {
  static String inputBuffer = "";
  
  // Read all available characters
  while (Serial1.available()) {
    char inChar = (char)Serial1.read();
    
    // Add character to buffer
    inputBuffer += inChar;
    
    // If we received a newline, process the complete message
    if (inChar == '\n') {
      // Remove any carriage return characters
      inputBuffer.replace("\r", "");
      
      // Process the complete line
      if (inputBuffer.length() > 0) {
        parseVisionData(inputBuffer);
      }
      
      // Clear buffer for next message
      inputBuffer = "";
    }
    
    // Prevent buffer overflow
    if (inputBuffer.length() > 100) {
      Serial.println("Warning: Serial buffer overflow, clearing...");
      inputBuffer = "";
    }
  }
}

void parseVisionData(String data) {
  // Format expected: TYPE,distance,height,x,y
  // Where TYPE is 'E' for enemy goal or 'H' for home goal
  
  Serial.print("Received data: ");
  Serial.println(data);
  
  // Check if data is valid
  if (data.length() < 5) {
    Serial.println("Data too short, ignoring");
    return;
  }
  
  // Get the first character to determine type
  char type = data.charAt(0);
  
  // Check if it's a valid type
  if (type != 'E' && type != 'H') {
    Serial.println("Invalid goal type, ignoring");
    return;
  }
  
  // Find comma positions for parsing
  int comma1 = data.indexOf(',', 0);
  int comma2 = data.indexOf(',', comma1 + 1);
  int comma3 = data.indexOf(',', comma2 + 1);
  int comma4 = data.indexOf(',', comma3 + 1);
  
  // Check if we have all required commas
  if (comma1 == -1 || comma2 == -1 || comma3 == -1 || comma4 == -1) {
    Serial.println("Missing commas in data, ignoring");
    return;
  }
  
  // Parse each field
  float distance = data.substring(comma1 + 1, comma2).toFloat();
  float height = data.substring(comma2 + 1, comma3).toFloat();  // Height instead of width
  int x = data.substring(comma3 + 1, comma4).toInt();
  int y = data.substring(comma4 + 1).toInt();
  
  // Validate parsed data
  if (distance <= 0 || height <= 0) {
    Serial.println("Invalid distance or height values, ignoring");
    return;
  }
  
  // Update the appropriate goal data
  if (type == 'E') {
    enemyGoal.detected = true;
    enemyGoal.distance = distance;
    enemyGoal.width = height;  // Now storing height in the width field for compatibility
    enemyGoal.x = x;
    enemyGoal.y = y;
    enemyGoal.lastUpdateTime = millis();
    
    Serial.print("Enemy goal updated: ");
    Serial.print(distance);
    Serial.print("cm at (");
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.println(")");
  } 
  else if (type == 'H') {
    homeGoal.detected = true;
    homeGoal.distance = distance;
    homeGoal.width = height;  // Now storing height in the width field for compatibility
    homeGoal.x = x;
    homeGoal.y = y;
    homeGoal.lastUpdateTime = millis();
    
    Serial.print("Home goal updated: ");
    Serial.print(distance);
    Serial.print("cm at (");
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.println(")");
  }
}

void checkDetectionTimeout() {
  unsigned long currentTime = millis();
  
  // Check if enemy goal detection has timed out
  if (enemyGoal.detected && (currentTime - enemyGoal.lastUpdateTime > DETECTION_TIMEOUT)) {
    enemyGoal.detected = false;
    Serial.println("Enemy goal detection timed out");
  }
  
  // Check if home goal detection has timed out
  if (homeGoal.detected && (currentTime - homeGoal.lastUpdateTime > DETECTION_TIMEOUT)) {
    homeGoal.detected = false;
    Serial.println("Home goal detection timed out");
  }
}

void printGoalStatus() {
  Serial.println("=== Goal Status ===");
  
  // Print enemy goal status
  Serial.print("Enemy Goal: ");
  if (enemyGoal.detected) {
    Serial.print("Detected at ");
    Serial.print(enemyGoal.distance);
    Serial.print("cm, Height: ");
    Serial.print(enemyGoal.width);  // Actually height now
    Serial.print("px, Position: (");
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
    Serial.print("cm, Height: ");
    Serial.print(homeGoal.width);  // Actually height now
    Serial.print("px, Position: (");
    Serial.print(homeGoal.x);
    Serial.print(",");
    Serial.print(homeGoal.y);
    Serial.println(")");
  } else {
    Serial.println("Not detected");
  }
  
  Serial.println("==================");
}

// Add your other robot control functions here
// void setupMotors() { ... }
// void aimTowardsGoal(GoalData goal) { ... }
// etc.