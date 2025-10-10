#include "GoalDetection.h"

GoalDetection::GoalDetection(int serialPort, unsigned long baudRate) {
    // Set serial port based on parameter
    switch (serialPort) {
        case 1:
            _serial = &Serial1;
            break;
        case 2:
            _serial = &Serial2;
            break;
        case 3:
            _serial = &Serial3;
            break;
        default:
            _serial = &Serial2; // Default to Serial2
    }
    
    _baudRate = baudRate;
    _debugEnabled = false;
    
    // Initialize goal data
    _enemyGoal = {false, 0, 0, 0, 0};
    _homeGoal = {false, 0, 0, 0, 0};
    
    _inputBuffer = "";
    _messageComplete = false;
}

void GoalDetection::begin() {
    // Initialize UART for communication with OpenMV
    _serial->begin(_baudRate);
    
    // Initialize USB serial for debugging if enabled
    if (_debugEnabled) {
        Serial.begin(9600);
        Serial.println("Teensy ready to receive OpenMV data via UART");
    }
    
    // Set up built-in LED for visual indication
    pinMode(LED_BUILTIN, OUTPUT);
}

void GoalDetection::update() {
    // Read data from UART
    readSerialData();
    
    // Process complete messages
    if (_messageComplete) {
        processMessage(_inputBuffer);
        _inputBuffer = "";
        _messageComplete = false;
    }
    
    // Check if detections are still valid (not timed out)
    checkDetectionTimeout();
    
    // Print current goal status if debugging
    if (_debugEnabled) {
        printGoalStatus();
    }
    
    // Visual indication when goals are detected
    digitalWrite(LED_BUILTIN, _enemyGoal.detected || _homeGoal.detected);
}

bool GoalDetection::isEnemyGoalDetected() const {
    return _enemyGoal.detected;
}

bool GoalDetection::isHomeGoalDetected() const {
    return _homeGoal.detected;
}

const GoalData& GoalDetection::getEnemyGoalData() const {
    return _enemyGoal;
}

const GoalData& GoalDetection::getHomeGoalData() const {
    return _homeGoal;
}

void GoalDetection::enableDebugOutput(bool enable) {
    _debugEnabled = enable;
    if (_debugEnabled) {
        Serial.begin(9600);
    }
}

void GoalDetection::readSerialData() {
    while (_serial->available() > 0) {
        char inChar = (char)_serial->read();
        
        // Add character to buffer until newline is received
        if (inChar == '\n') {
            _messageComplete = true;
            break;
        } else {
            _inputBuffer += inChar;
        }
    }
}

void GoalDetection::processMessage(const String& message) {
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
        _enemyGoal.detected = true;
        _enemyGoal.distance = distance;
        _enemyGoal.x = xPos;
        _enemyGoal.y = yPos;
        _enemyGoal.lastUpdateTime = millis();
        
        if (_debugEnabled) {
            Serial.print("Updated enemy goal: ");
            Serial.print(distance);
            Serial.print("cm, (");
            Serial.print(xPos);
            Serial.print(",");
            Serial.print(yPos);
            Serial.println(")");
        }
    } 
    else if (goalType == 'H') {
        _homeGoal.detected = true;
        _homeGoal.distance = distance;
        _homeGoal.x = xPos;
        _homeGoal.y = yPos;
        _homeGoal.lastUpdateTime = millis();
        
        if (_debugEnabled) {
            Serial.print("Updated home goal: ");
            Serial.print(distance);
            Serial.print("cm, (");
            Serial.print(xPos);
            Serial.print(",");
            Serial.print(yPos);
            Serial.println(")");
        }
    }
}

void GoalDetection::checkDetectionTimeout() {
    unsigned long currentTime = millis();
    
    // Check if enemy goal detection has timed out
    if (_enemyGoal.detected && (currentTime - _enemyGoal.lastUpdateTime > DETECTION_TIMEOUT)) {
        _enemyGoal.detected = false;
    }
    
    // Check if home goal detection has timed out
    if (_homeGoal.detected && (currentTime - _homeGoal.lastUpdateTime > DETECTION_TIMEOUT)) {
        _homeGoal.detected = false;
    }
}

void GoalDetection::printGoalStatus() {
    static unsigned long lastPrintTime = 0;
    unsigned long currentTime = millis();
    
    // Only print status every 500ms to avoid flooding serial console
    if (currentTime - lastPrintTime < 500) {
        return;
    }
    
    lastPrintTime = currentTime;
    
    // Print enemy goal status
    Serial.print("Enemy Goal: ");
    if (_enemyGoal.detected) {
        Serial.print("Detected at ");
        Serial.print(_enemyGoal.distance);
        Serial.print("cm, Position: (");
        Serial.print(_enemyGoal.x);
        Serial.print(",");
        Serial.print(_enemyGoal.y);
        Serial.println(")");
    } else {
        Serial.println("Not detected");
    }
    
    // Print home goal status
    Serial.print("Home Goal: ");
    if (_homeGoal.detected) {
        Serial.print("Detected at ");
        Serial.print(_homeGoal.distance);
        Serial.print("cm, Position: (");
        Serial.print(_homeGoal.x);
        Serial.print(",");
        Serial.print(_homeGoal.y);
        Serial.println(")");
    } else {
        Serial.println("Not detected");
    }
}