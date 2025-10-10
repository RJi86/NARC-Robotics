#ifndef VECTOR_MOTOR_CONTROL_H
#define VECTOR_MOTOR_CONTROL_H

#include <Arduino.h>
#include "maths/vector2.h"
#include "maths/vector3.h"
#include "maths/vector4.h"
#include "maths/matrix_4x3.h"
#include "maths/matrix_3x4.h"
#include "maths/matrix_3x3.h"

class VectorMotorControl {
public:
    VectorMotorControl();
    void begin();
    
    // Compatible with MotorClass interface
    void MoveDirection(float theta, float speed);
    void Rotation(int direction, float speed);
    void Stop();
    
    // Additional vector-based control functions
    void MoveWithRotation(float direction, float rotation, float speed);

private:
    struct motor_t {
        uint8_t pin_speed;
        uint8_t pin_dir;
        uint8_t pin_brake;
        
        void setup();
        void set_speed(int16_t speed);
    };
    
    // Motor configuration
    motor_t motor_1;
    motor_t motor_2;
    motor_t motor_3;
    motor_t motor_4;
    
    // Motor angles (in degrees)
    static const int MOTOR_ANGLE_1 = 45;   // Top-right
    static const int MOTOR_ANGLE_2 = 135;  // Top-left
    static const int MOTOR_ANGLE_3 = 225;  // Bottom-left
    static const int MOTOR_ANGLE_4 = 315;  // Bottom-right
    
    // Pre-computed transformation matrix
    matrix_4x3 mat_wheels;
    
    // Setup the transformation matrix
    void setup_matrix();
};

#endif // VECTOR_MOTOR_CONTROL_H