// Test 2: Arduino Due + one VMA330 IR obstacle sensor.

const int SENSOR_PIN = 26;

void setup() {
  Serial.begin(115200);
  pinMode(SENSOR_PIN, INPUT);
}

void loop() {
  int reading = digitalRead(SENSOR_PIN);

  if (reading == LOW) {
    Serial.println("Obstacle detected");
  } else {
    Serial.println("No obstacle detected");
  }

  delay(500);
}
