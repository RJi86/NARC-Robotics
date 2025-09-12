#ifndef GOAL_H
#define GOAL_H

struct GoalData {
  bool detected;
  float distance;  // Only distance now, no width
  int x;
  int y;
  unsigned long lastUpdateTime;
};

extern GoalData enemyGoal;
extern GoalData homeGoal;

#endif