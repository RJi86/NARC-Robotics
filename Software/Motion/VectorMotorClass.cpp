#include "VectorMotorControl.h"

VectorMotorControl::VectorMotorControl() {
    // Initialize motor pin configurations
    // Using the same pins as in your MotorClass example
    motor_1 = {6, 28, 29};   // BR_Motor - Bottom Right
    motor_2 = {7, 30, 31};   // BL_Motor - Bottom Left
    motor_3 = {8, 32, 33};   // TL_Motor - Top Left
    motor_4 = {9, 34, 35};   // TR_Motor - Top Right
}

void VectorMotorControl::begin() {
    // Set up each motor's pins
    motor_1.setup();
    motor_2.setup();
    motor_3.setup();
    motor_4.setup();
    
    // Initialize the transformation matrix
    setup_matrix();
}

void VectorMotorControl::setup_matrix() {
    // Calculate motor drive angles in radians
    float motor_drive_angles[4] = {
        ((MOTOR_ANGLE_1 + 90) % 360) * DEG_TO_RAD,
        ((MOTOR_ANGLE_2 + 90) % 360) * DEG_TO_RAD,
        ((MOTOR_ANGLE_3 + 90) % 360) * DEG_TO_RAD,
        ((MOTOR_ANGLE_4 + 90) % 360) * DEG_TO_RAD
    };
    
    // Create the motor-to-velocity mapping matrix
    matrix_type mat[3][4] = {
        { cosf(motor_drive_angles[0]), cosf(motor_drive_angles[1]), cosf(motor_drive_angles[2]), cosf(motor_drive_angles[3]) },
        { sinf(motor_drive_angles[0]), sinf(motor_drive_angles[1]), sinf(motor_drive_angles[2]), sinf(motor_drive_angles[3]) },
        { 1, 1, 1, 1 }
    };
    
    // Calculate the pseudoinverse to get velocity-to-motor mapping
    matrix_3x4 m(mat);
    matrix_4x3 m_t = matrix_3x4::transpose(m);
    matrix_3x3 mat3x3 = m_t * m;
    matrix_3x3 m_inv = matrix_3x3::inverse(mat3x3);
    auto m_near_final = m * m_inv;
    mat_wheels = matrix_3x4::transpose(m_near_final);
}

void VectorMotorControl::MoveDirection(float theta, float speed) {
    // Convert to vector components (x,y) and no rotation
    float rotation = 0.0f;
    MoveWithRotation(theta, rotation, speed);
}

void VectorMotorControl::Rotation(int direction, float speed) {
    // direction: 0 = clockwise, 1 = counterclockwise
    float rotation_value = direction == 0 ? 1.0f : -1.0f;
    
    // No directional movement, only rotation
    vector3 vec_dir(0, 0, rotation_value);
    auto motor_speeds = mat_wheels * vec_dir;
    
    // Set speeds to all motors
    motor_1.set_speed(motor_speeds[0] * speed);
    motor_2.set_speed(motor_speeds[1] * speed);
    motor_3.set_speed(motor_speeds[2] * speed);
    motor_4.set_speed(motor_speeds[3] * speed);
}

void VectorMotorControl::Stop() {
    motor_1.set_speed(0);
    motor_2.set_speed(0);
    motor_3.set_speed(0);
    motor_4.set_speed(0);
}

void VectorMotorControl::MoveWithRotation(float direction, float rotation, float speed) {
    // Create movement vector from direction angle
    Vector2f dir_vec(cosf(direction), sinf(direction));
    
    // Combine direction and rotation
    vector3 vec_dir(dir_vec.GetX(), dir_vec.GetY(), rotation);
    
    // Calculate motor speeds using the transformation matrix
    auto motor_speeds = mat_wheels * vec_dir;
    
    // Apply speed to all motors
    motor_1.set_speed(motor_speeds[0] * speed);
    motor_2.set_speed(motor_speeds[1] * speed);
    motor_3.set_speed(motor_speeds[2] * speed);
    motor_4.set_speed(motor_speeds[3] * speed);
}

// Implementation of motor_t methods
void VectorMotorControl::motor_t::setup() {
    pinMode(pin_speed, OUTPUT);
    pinMode(pin_dir, OUTPUT);
    pinMode(pin_brake, OUTPUT);
    
    analogWrite(pin_speed, 0);
    digitalWrite(pin_dir, LOW);
    digitalWrite(pin_brake, LOW);
}

void VectorMotorControl::motor_t::set_speed(int16_t speed) {
    uint16_t s = abs(speed) % 256;
    uint8_t dir = speed < 0 ? LOW : HIGH;
    
    analogWrite(pin_speed, s);
    digitalWrite(pin_dir, dir);
    digitalWrite(pin_brake, s == 0 ? HIGH : LOW);
}