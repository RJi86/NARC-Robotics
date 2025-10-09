#include "IRFilters.h"
#include <Arduino.h>

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
  270, 255, 240, 225, 210, 195, 180, 165,     // 0..7
  150, 135, 120, 105,  90,  75,  60,  45,     // 8..15
   30,  15,   0, 345, 330, 315, 300, 285      // 16..23  (no 360!)
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
    filters[i].threshold_on = 2;   // x consecutive ONs to confirm
    filters[i].threshold_off = 1;  // how many OFF to clear
    filters[i].max_count = 3; // window size
  }


}

static inline float toRad(float deg){ return deg * 0.017453292519943295f; }
static inline float toDeg(float rad){ return rad * 57.29577951308232f; }

// Small, simple circular-mean over your 24 booleans
// simplest possible average (no circular wrap handling)

// Use static inline to avoid multiple-definition if included in a header
static inline float averageAngleCircular(const bool detections[24]) {
  float sumX = 0.0f, sumY = 0.0f;
  int count = 0;
  for (int i = 0; i < 24; ++i) {
    if (!detections[i]) continue;
    const float th = toRad((float)SENSOR_ANGLES[i]);
    sumX += cosf(th);
    sumY += sinf(th);
    ++count;
  }
  if (count == 0) return NAN;
  float ang = toDeg(atan2f(sumY, sumX));
  if (ang < 0.0f) ang += 360.0f;
  return ang;
}

// usage inside findBallAngle():
// const float avg = averageAngleSimple(sensorDetections);
// if (isnan(avg)) Serial.println("Average angle: none");
// else            Serial.println(avg, 1);
// Function to read a specific sensor from a multiplexer
bool readSensor(Multiplexer &mux, int sensorIndex) {
  // Select channel (assumes A0=LSB, A1=mid, A2=MSB)
  mux.A0Val = (sensorIndex & 0x01);
  mux.A1Val = (sensorIndex & 0x02);
  mux.A2Val = (sensorIndex & 0x04);

  digitalWrite(mux.A0Pin, mux.A0Val);
  digitalWrite(mux.A1Pin, mux.A1Val);
  digitalWrite(mux.A2Pin, mux.A2Val);

  // Allow analog switch to settle
  delayMicroseconds(200);

  // Throw-away read to flush prior channel charge
  (void)digitalRead(mux.outputPin);

  // Short settle again (often not needed, but safe)
  delayMicroseconds(200);

  // Active-low sensor: 0 = detected
  return digitalRead(mux.outputPin) == LOW;
}


// Drop-in replacement: throttled printing but fast sensing
void findBallAngle(bool dumpSensors = true, uint16_t period_ms = 500) {
  static uint32_t last = 0;
  
  // Always read sensors (keeps filters responsive)
  bool sensorDetections[24] = {false};
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

  // Only print every period_ms
  if (millis() - last < period_ms) return;
  last = millis();

  const float avg = averageAngleCircular(sensorDetections);
  if (isnan(avg)) Serial.println("Average angle: none");
  else            Serial.print("Average angle: "), Serial.println(avg, 1);

  if (dumpSensors) {
    for (int i = 0; i < 24; i++) {
      Serial.print("S"); Serial.print(i); Serial.print("=");
      Serial.print(sensorDetections[i]);
      Serial.print(i == 23 ? '\n' : ' ');
    }
    Serial.println("==========");
  }
}



void loop() {
  // Print once every 700 ms; set to 1000 for once per second
  findBallAngle(/*dumpSensors=*/true, /*period_ms=*/700);
}


