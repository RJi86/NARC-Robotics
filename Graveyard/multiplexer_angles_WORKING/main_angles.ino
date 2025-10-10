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
  multiplexer1.A0Pin = 20;
  multiplexer1.A1Pin = 21;
  multiplexer1.A2Pin = 22;
  multiplexer1.outputPin = 2;
  
  // Set up multiplexer 2
  multiplexer2.A0Pin = 23;
  multiplexer2.A1Pin = 26;
  multiplexer2.A2Pin = 27;
  multiplexer2.outputPin = 3;
  
  // Set up multiplexer 3
  multiplexer3.A0Pin = 14;
  multiplexer3.A1Pin = 15;
  multiplexer3.A2Pin = 41;
  multiplexer3.outputPin = 4;
  
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

int findBallAngle() {
  bool sensorDetections[24] = {false};  // Store which sensors detect the ball
  int detectionCount = 0;               // Count how many sensors detect the ball
  
  // Read all sensors from multiplexer 1
  for (int i = 0; i < 8; i++) {
    sensorDetections[i] = readSensor(multiplexer1, i);
    if (sensorDetections[i]) detectionCount++;
  }
  
  // Read all sensors from multiplexer 2
  for (int i = 0; i < 8; i++) {
    sensorDetections[i + 8] = readSensor(multiplexer2, i);
    if (sensorDetections[i + 8]) detectionCount++;
  }
  
  // Read all sensors from multiplexer 3
  for (int i = 0; i < 8; i++) {
    sensorDetections[i + 16] = readSensor(multiplexer3, i);
    if (sensorDetections[i + 16]) detectionCount++;
  }
  
  // If no ball is detected, return -1
  if (detectionCount == 0) {
    return -1;
  }
  
  // Calculate the average angle of all sensors that detect the ball
  int totalAngle = 0;
  for (int i = 0; i < 24; i++) {
    if (sensorDetections[i]) {
      totalAngle += SENSOR_ANGLES[i];
    }
  }
  
  return totalAngle / detectionCount;  // Return the average angle
}

void loop() {
  int ballAngle = findBallAngle();
  
  if (ballAngle == -1) {
    Serial.println("No ball detected");
  } else {
    Serial.print("Ball detected at angle: ");
    Serial.print(ballAngle);
    Serial.println(" degrees");
  }
  
  delay(100);  // Small delay to avoid flooding the serial monitor
}