float average = 0.0;

void setup() {
  Serial.begin(112500);
  pinMode(20, INPUT_PULLUP);
  float total = 0.0;
  analogRead(20);
  for (int i = 0; i < 5; i++){
    total += float(analogRead(20));
    delay(20);
  }
  average = total/5;
  Serial.println(average);

}

void loop() {
  float captureReading = float(analogRead(20));
  Serial.println((captureReading));
  bool captured = false;
  if (captureReading > 2.15 * average){
    Serial.println("Caught ball");
    captured = true;
  }
  delay(20);
}