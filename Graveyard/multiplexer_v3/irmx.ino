#include "IRFilters.h"
static DigitalFilter filters[24]; //one per sensor

typedef struct {
  bool A0Val;
  bool A1Val;
  bool A2Val;
  byte A0Pin;
  byte A1Pin;
  byte A2Pin;
  int outputPin;
} Multiplexer;

// Define the angle for each sensor (in degrees)
// Assuming sensors are arranged in a circle with sensor 0 at 0 degrees,
// sensor 1 at 15 degrees, and so on
const int SENSOR_ANGLES[24] = {
  0, 15, 30, 45, 60, 75, 90, 105,         // Multiplexer 1 sensors (0-7)
  120, 135, 150, 165, 180, 195, 210, 225, // Multiplexer 2 sensors (8-15)
  240, 255, 270, 285, 300, 315, 330, 345  // Multiplexer 3 sensors (16-23)
};

Multiplexer multiplexer1;
Multiplexer multiplexer2;
Multiplexer multiplexer3;

void setup() {
  Serial.begin(115200);

  // Set up multiplexer 1
  multiplexer1.A0Pin = 38;
  multiplexer1.A1Pin = 37;
  multiplexer1.A2Pin = 36;
  multiplexer1.outputPin = 22;
  
  // Set up multiplexer 2
  multiplexer2.A0Pin = 35;
  multiplexer2.A1Pin = 34;
  multiplexer2.A2Pin = 33;
  multiplexer2.outputPin = 23;
  
  // Set up multiplexer 3
  multiplexer3.A0Pin = 27;
  multiplexer3.A1Pin = 26;
  multiplexer3.A2Pin = 25;
  multiplexer3.outputPin = 24;
  
  // Set all multiplexer pins as OUTPUT
  pinMode(multiplexer1.A0Pin, OUTPUT);
  pinMode(multiplexer1.A1Pin, OUTPUT);
  pinMode(multiplexer1.A2Pin, OUTPUT);
  
  pinMode(multiplexer2.A0Pin, OUTPUT);
  pinMode(multiplexer2.A1Pin, OUTPUT);
  pinMode(multiplexer2.A2Pin, OUTPUT);
  
  pinMode(multiplexer3.A0Pin, OUTPUT);
  pinMode(multiplexer3.A1Pin, OUTPUT);
  pinMode(multiplexer3.A2Pin, OUTPUT);
  
  // Set all output pins as INPUT
  pinMode(multiplexer1.outputPin, INPUT);
  pinMode(multiplexer2.outputPin, INPUT);
  pinMode(multiplexer3.outputPin, INPUT);

  for (int i = 0; i < 24; i++) {
    filters[i].threshold_on = 5;   // x consecutive ONs to confirm
    filters[i].threshold_off = 1;  // 1 OFF to clear
    filters[i].max_count = 5;
  }


}

// Function to read a specific sensor from a multiplexer
bool readSensor(Multiplexer &mux, int sensorIndex) {
  // Set address pins based on sensor index (0-7)
  mux.A0Val = (sensorIndex & 0x01) > 0;  // Bit 0
  mux.A1Val = (sensorIndex & 0x02) > 0;  // Bit 1
  mux.A2Val = (sensorIndex & 0x04) > 0;  // Bit 2
  
  // Set the address pins
  digitalWrite(mux.A0Pin, mux.A0Val);
  digitalWrite(mux.A1Pin, mux.A1Val);
  digitalWrite(mux.A2Pin, mux.A2Val);
  
  // Read the output (0 = ball detected, 1 = no detection)
  return digitalRead(mux.outputPin) == 0;  // Return true if ball is detected
}

void findBallAngle() {
  bool sensorDetections[24] = {false};  // Store which sensors detect the ball


  for (int i = 0; i < 8; i++) {
    bool raw1 = readSensor(multiplexer1, i);
    sensorDetections[i] = filters[i].process(raw1);
  }

  for (int i = 0; i < 8; i++) {
    bool raw2 = readSensor(multiplexer2, i);
    sensorDetections[i + 8] = filters[i + 8].process(raw2);
  }

  for (int i = 0; i < 8; i++) {
    bool raw3 = readSensor(multiplexer3, i);
    sensorDetections[i + 16] = filters[i + 16].process(raw3);
  }

  for (int i = 0; i < 24; i++){
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(sensorDetections[i]);
  }
}


void loop() {
  findBallAngle();

  
  delay(30);  // Small delay to avoid flooding the serial monitor
  Serial.println("==========");
}
