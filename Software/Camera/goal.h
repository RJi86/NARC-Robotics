#ifndef GOAL_H
#define GOAL_H

// Structure to hold goal detection data
struct GoalData {
    bool detected;          // Whether goal is currently detected
    bool inFront;           // Whether goal is in front (true) or rear (false)
    int height;             // Height in pixels
    int x;                  // X position on camera/sensor
    int y;                  // Y position on camera/sensor
    unsigned long lastUpdateTime;  // Last time this data was updated
};

#endif // GOAL_H