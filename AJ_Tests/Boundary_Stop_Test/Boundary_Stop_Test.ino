// AJ Donald 101259149 

// Arduino Due (Programming Port). Serial 115200: f forward; s stop.
// Sensor VCC -> 3.3V, GND -> GND, OUT1 -> A0. One sensor for this test.

const int DIR_PINS[] = {7, 9, 11, 13};
const int EN_PINS[] = {6, 8, 10, 12};
const int FORWARD_LEVEL[] = {LOW, LOW, HIGH, HIGH};
const int SENSOR_PIN = A0;
const int BLACK_THRESHOLD = 3000;
const int TEST_PWM = 100; // Lower initial duty not a measured speed
const unsigned long MAX_RUN_MS = 3000;
bool running = false;
unsigned long startedAt = 0;
unsigned long stoppedAt = 0;
unsigned long lastPrint = 0;

void stopMotors(const char *reason) {
  for (int i = 0; i < 4; i++) analogWrite(EN_PINS[i], 0);
  running = false;
  stoppedAt = millis();
  Serial.println(reason);
}

void setup() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(EN_PINS[i], LOW);
    pinMode(EN_PINS[i], OUTPUT);
    digitalWrite(DIR_PINS[i], LOW);
    pinMode(DIR_PINS[i], OUTPUT);
  }
  analogWriteResolution(8);
  analogReadResolution(12);
  pinMode(SENSOR_PIN, INPUT);
  Serial.begin(115200);
}

void loop() {
  int reading = analogRead(SENSOR_PIN);
  bool onBlack = reading > BLACK_THRESHOLD;
  unsigned long now = millis();
  if (running && onBlack) stopMotors("STOP: black tape detected.");
  else if (running && now - startedAt >= MAX_RUN_MS)
    stopMotors("STOP: 3-second limit.");

  if (Serial.available()) {
    char command = Serial.read();
    if (command == 's') stopMotors("STOP: manual command.");
    else if (command == 'f') {
      if (onBlack) Serial.println("Start blocked: sensor sees black tape.");
      else if (running) Serial.println("Already running.");
      else if (now - stoppedAt < 1000)
        Serial.println("Wait 1 second after stopping.");
      else {
        for (int i = 0; i < 4; i++) digitalWrite(DIR_PINS[i], FORWARD_LEVEL[i]);
        startedAt = now;
        for (int i = 0; i < 4; i++) analogWrite(EN_PINS[i], TEST_PWM);
        running = true;
        Serial.println("FORWARD: waiting for black tape.");
      }
    }
  }
  if (now - lastPrint >= 250 && Serial.availableForWrite() >= 60) {
    lastPrint = now;
    Serial.print("Reading: "); Serial.print(reading);
    Serial.print(onBlack ? " | BLACK TAPE" : " | FLOOR");
    Serial.println(running ? " | RUNNING" : " | STOPPED");
  }
}
