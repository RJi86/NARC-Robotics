#ifndef GOAL_DETECTION_H
#define GOAL_DETECTION_H

#include "goal.h"
#include <Arduino.h>

// Goal Detection Class
class GoalDetection {
public:
    // Constructor
    GoalDetection(int serialPort = 2, unsigned long baudRate = 115200);
    
    // Initialize the detection system
    void begin();
    
    // Process incoming data (call in loop)
    void update();
    
    // Get the current enemy goal status
    bool isEnemyGoalDetected() const;
    
    // Get whether enemy goal is in front (true) or rear (false)
    bool isEnemyGoalInFront() const;
    
    // Get the current home goal status
    bool isHomeGoalDetected() const;
    
    // Get whether home goal is in front (true) or rear (false)
    bool isHomeGoalInFront() const;
    
    // Get detailed goal data
    const GoalData& getEnemyGoalData() const;
    const GoalData& getHomeGoalData() const;
    
    // Debug functions
    void enableDebugOutput(bool enable);
    
private:
    // Serial port configuration
    HardwareSerial* _serial;
    unsigned long _baudRate;
    bool _debugEnabled;
    
    // Goal detection data
    GoalData _enemyGoal;
    GoalData _homeGoal;
    
    // Constants
    static const unsigned long DETECTION_TIMEOUT = 500; // Consider detection lost after 500ms
    
    // Buffer for incoming serial data
    String _inputBuffer;
    bool _messageComplete;
    
    // Private methods
    void readSerialData();
    void processMessage(const String& message);
    void checkDetectionTimeout();
    void printGoalStatus();
};

#endif // GOAL_DETECTION_H