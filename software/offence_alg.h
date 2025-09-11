#pragma once
#include <Arduino.h>
#include "controllers/IRSensorController.h"
#include "controllers/LightSensorController.h"
#include "controllers/DirectionUtils.h"
#include "config/OffenseConfig.h"
#include "sensors/Gyro.h"
#include "motion/MotorClass.h"

class OffenseController {
public:
  enum State : uint8_t { SEARCH=0, APPROACH, ALIGN_BALL, ALIGN_GOAL, DRAG_GOAL };

  OffenseController(IRSensorController& ir,
                    LightSensorController& lt,
                    Gyro& gyro,
                    MotorClass& drive)
  : m_ir(ir), m_lt(lt), m_gyro(gyro), m_drive(drive) {}

  void begin(){ m_state = SEARCH; }

  void step(){
    m_ir.update();
    m_lt.update();
    m_gyro.update();

    auto irB = m_ir.bearing();
    auto ltB = m_lt.bearing();

    // Override rule: light significant -> move opposite of light
    bool lightOverride = (ltB.confidence >= offensecfg::kLightOverrideThresh);

    // Heuristic proximity (close to ball)
    bool ballClose = irB.confidence >= offensecfg::kBallCloseThresh;

    switch(m_state){
      case SEARCH: {
        float th = lightOverride ? DirectionUtils::wrap2pi(ltB.bearing_rad + PI) : irB.bearing_rad;
        float sp = lightOverride ? offensecfg::kApproachSpeed : offensecfg::kSearchSpeed;
        m_drive.MoveDirection(th, sp);
        if (irB.confidence > 0.2f) m_state = APPROACH;
      } break;
case APPROACH: {
        float th = lightOverride ? DirectionUtils::wrap2pi(ltB.bearing_rad + PI) : irB.bearing_rad;
        m_drive.MoveDirection(th, offensecfg::kApproachSpeed);
        float err = DirectionUtils::angdiff(irB.bearing_rad, 0.0f);
        float omega = constrain(offensecfg::kRotKpBall * (-err) / PI, -1.0f, 1.0f);
        m_drive.MoveVector(cosf(th)offensecfg::kApproachSpeed, sinf(th)offensecfg::kApproachSpeed, omega);
        if (ballClose) m_state = ALIGN_BALL;
      } break;

      case ALIGN_BALL: {
        float err = DirectionUtils::angdiff(irB.bearing_rad, 0.0f);
        float omega = constrain(offensecfg::kRotKpBall * (-err) / PI, -1.0f, 1.0f);
        m_drive.MoveVector(0, 0, omega);
        if (fabsf(err) < (5.0f * PI/180.0f)) m_state = ALIGN_GOAL;
      } break;

      case ALIGN_GOAL: {
        float yaw = m_gyro.headingRad();
        float err = DirectionUtils::angdiff(offensecfg::kEnemyGoalHeadingRad, yaw);
        float omega = constrain(offensecfg::kRotKpGoal * (err) / PI, -1.0f, 1.0f);
        m_drive.MoveVector(0, 0, omega);
        if (fabsf(err) < (5.0f * PI/180.0f)) m_state = DRAG_GOAL;
      } break;

      case DRAG_GOAL: {
        float yaw = m_gyro.headingRad();
        float goalErr = DirectionUtils::angdiff(offensecfg::kEnemyGoalHeadingRad, yaw);
        float omega = constrain(offensecfg::kRotKpGoal * (goalErr) / PI, -1.0f, 1.0f);
        m_drive.MoveVector(offensecfg::kDragSpeed, 0.0f, omega); // forward
        if (!ballClose) m_state = APPROACH; // lost ball -> go back
      } break;
    }
  }

  State state() const { return m_state; }

private:
  IRSensorController&   m_ir;
  LightSensorController& m_lt;
  Gyro&                 m_gyro;
  MotorClass&           m_drive;
  State                 m_state{SEARCH};
};
