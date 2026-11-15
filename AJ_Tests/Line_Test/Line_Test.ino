// Test 3: Arduino Due + ONE output of a line-sensor module.

const int SENSOR_PIN = A0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
}

void loop() {
  int reading = analogRead(SENSOR_PIN);

  Serial.print("Line sensor reading: ");
  Serial.println(reading);

  delay(500);
}
