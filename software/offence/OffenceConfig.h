#pragma once
#include <Arduino.h>
#include "ClusterConfig.h"

namespace offencecfg {
constexpr float kMaxSpeed = 1.0f; // 0..1 normalized translational speed
constexpr float kApproachSpeed = 0.8f; // approach ball
constexpr float kDragSpeed = 0.6f; // push ball to goal
constexpr float kSearchSpeed = 0.3f; // when no target
constexpr float kMaxOmega = 2.5f; // rad/s equivalent for rotation scale
constexpr float kRotKpBall = 2.0f; // rotate to face ball
constexpr float kRotKpGoal = 2.0f; // rotate to face goal

constexpr float kLightOverrideThresh = 0.70f; // confidence to override IR
constexpr float kBallCloseThresh = 0.65f; // heuristic proximity via IR strength

// Field setup: absolute heading of enemy goal, in robot yaw frame (set at init)
// 0 = robot facing positive X at boot and enemy goal straight ahead
constexpr float kEnemyGoalHeadingRad = 0.0f;
}