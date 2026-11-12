// Arduino Due (Programming Port). Existing Cytron wiring unchanged.
// Serial 115200: f forward, b backward, l left, r right, s stop.
// IMPORTANT: r now means RIGHT, not backward. Each move lasts 1 second.
const int DIR_PINS[] = {7, 9, 11, 13}; // Cytron 1, 3, 5, 7
const int EN_PINS[]  = {6, 8, 10, 12}; // Cytron 2, 4, 6, 8

// Entries are M1, M2, M3, M4. Copy from your working forward/back sketch.
const int FORWARD_LEVEL[] = {LOW, LOW, HIGH, HIGH};
// Viewed from behind the robot, facing its front: true = left side.
// Assumption: M1/M2 left; M3/M4 right. Change to match your wiring.
const bool IS_LEFT[] = {true, true, false, false};
const int TEST_PWM = 150; // Duty setting, not measured speed.
const unsigned long RUN_MS = 1000;
bool running = false;
unsigned long startedAt = 0;
unsigned long stoppedAt = 0;

void stopMotors() {
  for (int i = 0; i < 4; i++) analogWrite(EN_PINS[i], 0);
  running = false;
  stoppedAt = millis();
  Serial.println("Stopped. Wait 1 second before the next move.");
}

// Start a timed move. Reject direction changes until stopped for 1 second.
void driveSides(bool leftForward, bool rightForward) {
  if (running || millis() - stoppedAt < 1000) {
    Serial.println("Wait until stopped for 1 second.");
    return;
  }
  for (int i = 0; i < 4; i++) {
    bool forward = IS_LEFT[i] ? leftForward : rightForward;
    int direction = FORWARD_LEVEL[i];
    if (!forward) direction = (direction == LOW) ? HIGH : LOW;
    digitalWrite(DIR_PINS[i], direction);
  }
  startedAt = millis();
  for (int i = 0; i < 4; i++) analogWrite(EN_PINS[i], TEST_PWM);
  running = true;
}

void moveForward()  { driveSides(true, true); }
void moveBackward() { driveSides(false, false); }
void turnLeft()     { driveSides(false, true); } // Left backward, right forward.
void turnRight()    { driveSides(true, false); } // Left forward, right backward.

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
  switch (Serial.read()) {
    case 'f': moveForward(); break;
    case 'b': moveBackward(); break;
    case 'l': turnLeft(); break;
    case 'r': turnRight(); break;
    case 's': stopMotors(); break;
  }
}
