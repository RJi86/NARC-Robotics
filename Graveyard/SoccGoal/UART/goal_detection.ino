#include "goal.h"

// UART Configuration
#define SERIAL_BAUD_RATE 115200

// Goal detection data
GoalData enemyGoal = {false, 0, 0, 0, 0};
GoalData homeGoal = {false, 0, 0, 0, 0};

// Constants
const unsigned long DETECTION_TIMEOUT = 500; // Consider detection lost after 500ms

// Buffer for incoming serial data
String inputBuffer = "";
bool messageComplete = false;

void setup() {
  // Initialize UART for communication with OpenMV
  Serial1.begin(SERIAL_BAUD_RATE);
  
  // Initialize USB serial for debugging
  Serial.begin(9600);
  
  // Set up built-in LED for visual indication
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.println("Teensy ready to receive OpenMV data via UART");
}

void loop() {
  // Read data from UART
  readSerialData();
  
  // Process complete messages
  if (messageComplete) {
    processMessage(inputBuffer);
    inputBuffer = "";
    messageComplete = false;
  }
  
  // Check if detections are still valid (not timed out)
  checkDetectionTimeout();
  
  // Print current goal status
  printGoalStatus();
  
  // Visual indication when goals are detected
  digitalWrite(LED_BUILTIN, enemyGoal.detected || homeGoal.detected);
  
  delay(10); // Short delay to reduce CPU usage
}

void readSerialData() {
  while (Serial1.available() > 0) {
    char inChar = (char)Serial1.read();
    
    // Add character to buffer until newline is received
    if (inChar == '\n') {
      messageComplete = true;
      break;
    } else {
      inputBuffer += inChar;
    }
  }
}

void processMessage(String message) {
  // Parse message in format: TYPE,distance,height,x,y
  // TYPE is 'E' for enemy, 'H' for home
  
  // Check if the message is in the expected format
  if (message.length() < 5) {
    return; // Message too short, ignore
  }
  
  // Extract goal type
  char goalType = message.charAt(0);
  
  // Check if a valid goal type
  if (goalType != 'E' && goalType != 'H') {
    return; // Invalid goal type, ignore
  }
  
  // Find commas for parsing
  int firstComma = message.indexOf(',');
  int secondComma = message.indexOf(',', firstComma + 1);
  int thirdComma = message.indexOf(',', secondComma + 1);
  int fourthComma = message.indexOf(',', thirdComma + 1);
  
  // Check if all commas were found
  if (firstComma == -1 || secondComma == -1 || thirdComma == -1 || fourthComma == -1) {
    return; // Missing commas, ignore message
  }
  
  // Extract values
  float distance = message.substring(firstComma + 1, secondComma).toFloat();
  int height = message.substring(secondComma + 1, thirdComma).toInt();
  int xPos = message.substring(thirdComma + 1, fourthComma).toInt();
  int yPos = message.substring(fourthComma + 1).toInt();
  
  // Update the appropriate goal data
  if (goalType == 'E') {
    enemyGoal.detected = true;
    enemyGoal.distance = distance;
    enemyGoal.x = xPos;
    enemyGoal.y = yPos;
    enemyGoal.lastUpdateTime = millis();
    
    Serial.print("Updated enemy goal: ");
    Serial.print(distance);
    Serial.print("cm, (");
    Serial.print(xPos);
    Serial.print(",");
    Serial.print(yPos);
    Serial.println(")");
  } 
  else if (goalType == 'H') {
    homeGoal.detected = true;
    homeGoal.distance = distance;
    homeGoal.x = xPos;
    homeGoal.y = yPos;
    homeGoal.lastUpdateTime = millis();
    
    Serial.print("Updated home goal: ");
    Serial.print(distance);
    Serial.print("cm, (");
    Serial.print(xPos);
    Serial.print(",");
    Serial.print(yPos);
    Serial.println(")");
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
  static unsigned long lastPrintTime = 0;
  unsigned long currentTime = millis();
  
  // Only print status every 500ms to avoid flooding serial console
  if (currentTime - lastPrintTime < 500) {
    return;
  }
  
  lastPrintTime = currentTime;
  
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