#ifndef GOAL_H
#define GOAL_H

// Structure to hold goal detection data
struct GoalData {
    bool detected;          // Whether goal is currently detected
    float distance;         // Distance to goal in cm
    int x;                  // X position on camera/sensor
    int y;                  // Y position on camera/sensor
    unsigned long lastUpdateTime;  // Last time this data was updated
};

#endif // GOAL_H