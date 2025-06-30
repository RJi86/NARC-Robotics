#include <math.h>
#include <SoftwareSerial.h>

const float pi = 3.14159265358979323846;

// const int dirPin0 = 4;
// const int pwmPin0 = 3;
// const int brakePin0 = 5;

// const int dirpPin1 = 8;
// const int pwmPin1 = 7;
// const int brakePin1 = 9;

// const int dirpPin2 = 12;
// const int pwmPin2 = 11;
// const int brakePin2 = 13;

// const int dirpPin3 = 16;
// const int pwmPin3 = 15;
// const int brakePin3 = 17;

float value = sqrt(2)/4;

int pins[4][3] = {
  {4, 3, 5}
  {8, 7, 9}
  {12, 11, 13}
  {16, 15, 17}
}

float speedjust[4][3] = {
  {-value, value, 0.25}
  {-value, -value, 0.25}
  {value, -value, 0.25}
  {value, value, 0.25}
}

void setup() {
  for (byte i = 0; i<4; i++){
    pinMode(pins[i][0], OUTPUT);
    pinMode(pins[i][1], OUTPUT);
    pinMode(pins[i][2], OUTPUT);
    digitalWrite(pins[i][2], LOW);
  }

  // pinMode(dirPin0, OUTPUT);
  // pinMode(pwmPin0, OUTPUT);
  // pinMode(brakePin0, OUTPUT);
  // pinMode(dirPin1, OUTPUT);
  // pinMode(pwmPin1, OUTPUT);
  // pinMode(brakePin1, OUTPUT);
  // pinMode(dirPin2, OUTPUT);
  // pinMode(pwmPin2, OUTPUT);
  // pinMode(brakePin2, OUTPUT);
  // pinMode(dirPin3, OUTPUT);
  // pinMode(pwmPin3, OUTPUT);
  // pinMode(brakePin3, OUTPUT);

  // digitalWrite(brakePin0, LOW); // Disable brake so motor can move
  // digitalWrite(brakePin1, LOW);
  // digitalWrite(brakePin2, LOW);
  // digitalWrite(brakePin3, LOW);
}

void loop() {
  // Move forward
  digitalWrite(dirPin0, HIGH);
  analogWrite(pwmPin0, 180); // PWM value between 0–255

  float theta = 0.7853981634;
  float motion[4];
  float x = 1;
  float y = tan(theta);
  float w = 0; // no rotation
  if (theta > pi/4){
    y = 1;
    x = 1/tan(theta);
  }

  for (byte i = 0; i < 4; i++) {
    motion[i] = speedjust[i][0] * x + speedjust[i][1] * y + speedjust[i][2] * w;
    motion[i] *= 255;  // Scale to Pin range (-255 to 255)
  }

  if (motion[0] < 0) {
    digitalWrite(dirPin0, LOW); // move backward if velocity is negative
  }
  if (motion[1] < 0) {
    digitalWrite(dirPin1, LOW); // move backward if velocity is negative
  }
  if (motion[2] < 0) {
    digitalWrite(dirPin2, LOW); // move backward if velocity is negative
  }
  if (motion[3] < 0) {
    digitalWrite(dirPin3, LOW); // move backward if velocity is negative
  }

}