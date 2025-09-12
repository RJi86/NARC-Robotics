#include <Wire.h>
#include "functions/analog.h"
#include "functions/analog_multi.h"
#include "functions/pins.h"
#include "IRcluster.h"
#include "ClusterConfig.h"

#define THRESHOLD 120 // threshold value (adjustable)

// Define the three multiplexers using the pins from Pins.h
analog_multi_t IRcluster_1(MUX0_SIG, MUX0_S0, MUX0_S1, MUX0_S2);
analog_multi_t IRcluster_2(MUX1_SIG, MUX1_S0, MUX1_S1, MUX1_S2);
analog_multi_t IRcluster_3(MUX2_SIG, MUX2_S0, MUX2_S1, MUX2_S2);

// Arrays to store sensor readings
uint16_t cluster1_values[8] = {0};
uint16_t cluster2_values[8] = {0};
uint16_t cluster3_values[8] = {0};
uint16_t all_sensor_values[24] = {0}; // Combined array for all sensors

unsigned long lastPrintTime = 0;
const unsigned long printInterval = 500; // Print every 500ms

void setup() {
  Serial.begin(115200);
  Serial.println(F("IR Cluster Multiplexer Test"));
  Serial.println(F("----------------------------"));
  delay(1000);
}

void loop() {
  // Read all values from each multiplexer
  readAllClusters();
  
  // Print values at regular intervals
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= printInterval) {
    printSensorValues();
    lastPrintTime = currentTime;
  }
}

void readAllClusters() {
  // Read all 8 values from each multiplexer
  for (uint8_t i = 0; i < 8; i++) {
    cluster1_values[i] = IRcluster_1.read(i);
    cluster2_values[i] = IRcluster_2.read(i);
    cluster3_values[i] = IRcluster_3.read(i);
    
    // Store in the combined array
    all_sensor_values[i] = cluster1_values[i];
    all_sensor_values[i + 8] = cluster2_values[i];
    all_sensor_values[i + 16] = cluster3_values[i];
  }
}

void printSensorValues() {
  // Print header
  Serial.println(F("\n--- IR Sensor Values ---"));
  
  // Print values for each cluster
  Serial.println(F("Cluster 1:"));
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print(F("Channel "));
    Serial.print(i);
    Serial.print(F(": "));
    Serial.println(cluster1_values[i]);
  }
  
  Serial.println(F("\nCluster 2:"));
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print(F("Channel "));
    Serial.print(i);
    Serial.print(F(": "));
    Serial.println(cluster2_values[i]);
  }
  
  Serial.println(F("\nCluster 3:"));
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print(F("Channel "));
    Serial.print(i);
    Serial.print(F(": "));
    Serial.println(cluster3_values[i]);
  }
  
  // Print highest reading
  uint16_t maxVal = 0;
  uint8_t maxIdx = 0;
  for (uint8_t i = 0; i < 24; i++) {
    if (all_sensor_values[i] > maxVal) {
      maxVal = all_sensor_values[i];
      maxIdx = i;
    }
  }
  Serial.print(F("\nHighest reading: Sensor "));