// Arduino Due (Programming Port), Cytron FD04A.
// Same wiring as Four_Motor_Test. Battery -> Cytron; USB -> Due.
// Due GND -> Cytron G. Leave Cytron V disconnected.
// Serial Monitor 115200: f = forward, b (or r) = backward, s = stop.
// Each movement lasts one second. Check directions with wheels raised first.
const int DIR_PINS[] = {7, 9, 11, 13};  // Cytron labels 1, 3, 5, 7
const int EN_PINS[]  = {6, 8, 10, 12};  // Cytron labels 2, 4, 6, 8

// M1, M2, M3, M4: electrical direction that makes each wheel drive forward.
// Initial guess assumes M1/M2 on one side, M3/M4 on the other.
// Change LOW <-> HIGH for any wheel that turns backward during f.
const int FORWARD_LEVEL[] = {LOW, LOW, HIGH, HIGH};
const int TEST_PWM = 150;  // Duty setting 0-255; NOT measured speed.
const unsigned long RUN_MS = 1000;
bool running = false;
unsigned long startedAt = 0;
unsigned long stoppedAt = 0;

void stopMotors() {
  for (int i = 0; i < 4; i++) analogWrite(EN_PINS[i], 0);
  running = false;
  stoppedAt = millis();
  Serial.println("Stopped. Wait 1 second before the next command.");
}

void setup() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(EN_PINS[i], LOW);
    pinMode(EN_PINS[i], OUTPUT);
    digitalWrite(DIR_PINS[i], LOW);
    pinMode(DIR_PINS[i], OUTPUT);
  }
  analogWriteResolution(8);
  Serial.begin(115200);
}

void loop() {
  if (running && millis() - startedAt >= RUN_MS) stopMotors();
  if (!Serial.available()) return;
  char command = Serial.read();
  if (command == 's') {
    stopMotors();
  } else if (command == 'f' || command == 'b' || command == 'r') {
    if (running || millis() - stoppedAt < 1000) {
      Serial.println("Wait until stopped for 1 second.");
      return;
    }
    // Set every direction before enabling any motor.
    for (int i = 0; i < 4; i++) {
      int direction = FORWARD_LEVEL[i];
      if (command != 'f') direction = (direction == LOW) ? HIGH : LOW;
      digitalWrite(DIR_PINS[i], direction);
    }
    startedAt = millis();
    for (int i = 0; i < 4; i++) analogWrite(EN_PINS[i], TEST_PWM);
    running = true;
    Serial.println(command == 'f' ? "Forward: 1 second." : "Backward: 1 second.");
  }
}
