#pragma once
#include <Arduino.h>
#include "controllers/IRSensorController.h"
#include "controllers/LightSensorController.h"
#include "controllers/angle_utils.h"
#include "Offence/OffenceConfig.h"
#include "controllers/Gyro.h"
#include "omni_motion/motors/MotorClass.h"

class OffenceController {
public:
  enum State : uint8_t { SEARCH=0, APPROACH, ALIGN_BALL, ALIGN_GOAL, DRAG_GOAL };

  OffenceController(IRSensorController& ir,
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
    bool lightOverride = (ltB.confidence >= offencecfg::kLightOverrideThresh);

    // Heuristic proximity (close to ball)
    bool ballClose = irB.confidence >= offencecfg::kBallCloseThresh;

    switch(m_state){
      case SEARCH: {
        float th = lightOverride ? DirectionUtils::wrap2pi(ltB.bearing_rad + PI) : irB.bearing_rad;
        float sp = lightOverride ? offencecfg::kApproachSpeed : offencecfg::kSearchSpeed;
        m_drive.MoveDirection(th, sp);
        if (irB.confidence > 0.2f) m_state = APPROACH;
      } break;

      case APPROACH: {
        float th = lightOverride ? DirectionUtils::wrap2pi(ltB.bearing_rad + PI) : irB.bearing_rad;
        m_drive.MoveDirection(th, offencecfg::kApproachSpeed);
        float err = DirectionUtils::angdiff(irB.bearing_rad, 0.0f);
        float omega = constrain(offencecfg::kRotKpBall * (-err) / PI, -1.0f, 1.0f);
        m_drive.MoveVector(cosf(th)*offencecfg::kApproachSpeed, sinf(th)*offencecfg::kApproachSpeed, omega);
        if (ballClose) m_state = ALIGN_BALL;
      } break;

      case ALIGN_BALL: {
        float err = DirectionUtils::angdiff(irB.bearing_rad, 0.0f);
        float omega = constrain(offencecfg::kRotKpBall * (-err) / PI, -1.0f, 1.0f);
        m_drive.MoveVector(0, 0, omega);
        if (fabsf(err) < (5.0f * PI/180.0f)) m_state = ALIGN_GOAL;
      } break;

      case ALIGN_GOAL: {
        float yaw = m_gyro.headingRad();
        float err = DirectionUtils::angdiff(offencecfg::kEnemyGoalHeadingRad, yaw);
        float omega = constrain(offencecfg::kRotKpGoal * (err) / PI, -1.0f, 1.0f);
        m_drive.MoveVector(0, 0, omega);
        if (fabsf(err) < (5.0f * PI/180.0f)) m_state = DRAG_GOAL;
      } break;

      case DRAG_GOAL: {
        float th = lightOverride ? DirectionUtils::wrap2pi(ltB.bearing_rad + PI) : irB.bearing_rad;
        float sp = lightOverride ? offencecfg::kApproachSpeed : offencecfg::kSearchSpeed;
        m_drive.MoveDirection(th, sp);
        float yaw = m_gyro.headingRad();
        float goalErr = DirectionUtils::angdiff(offencecfg::kEnemyGoalHeadingRad, yaw);
        float omega = constrain(offencecfg::kRotKpGoal * (goalErr) / PI, -1.0f, 1.0f);
        m_drive.MoveVector(offencecfg::kDragSpeed, 0.0f, omega); // forward
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

