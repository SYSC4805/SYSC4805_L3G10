// Arduino Due, Programming Port. Serial Monitor: 115200 baud.
// l = spin left for 1 second; r = spin right; s = stop.
// Motor-only test: sensors are not read. Starts stopped.
const int DIR_PINS[4] = {7, 9, 11, 13};
const int EN_PINS[4] = {6, 8, 10, 12};
const int FORWARD_LEVEL[4] = {LOW, LOW, HIGH, HIGH};
const bool IS_LEFT[4] = {true, true, false, false};
const int TURN_PWM = 100; // Same turning power as the combined sketch.
const unsigned long TURN_MS = 1000; // Duration, not a measured angle.
bool turning = false;
unsigned long startedAt = 0;

void stopMotors() {
  for (int i = 0; i < 4; i++) analogWrite(EN_PINS[i], 0);
  turning = false;
}
void startTurn(bool right) {
  stopMotors();
  for (int i = 0; i < 4; i++) {
    bool forward = IS_LEFT[i] ? right : !right;
    int direction = forward ? FORWARD_LEVEL[i] : !FORWARD_LEVEL[i];
    digitalWrite(DIR_PINS[i], direction);
  }
  for (int i = 0; i < 4; i++) analogWrite(EN_PINS[i], TURN_PWM);
  startedAt = millis();
  turning = true;
  Serial.println(right ? "RIGHT: left wheels forward, right wheels backward" :
                         "LEFT: left wheels backward, right wheels forward");
}
void setup() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(EN_PINS[i], LOW);
    pinMode(EN_PINS[i], OUTPUT);
    digitalWrite(DIR_PINS[i], LOW);
    pinMode(DIR_PINS[i], OUTPUT);
  }
  analogWriteResolution(8);
  stopMotors();
  Serial.begin(115200);
  Serial.println("TURN TEST: l = left, r = right, s = stop. Each turn lasts 1 second.");
}
void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 's') {
      stopMotors();
      Serial.println("STOPPED");
    } else if (!turning && (command == 'l' || command == 'r')) {
      startTurn(command == 'r');
    }
  }
  if (turning && millis() - startedAt >= TURN_MS) {
    stopMotors();
    Serial.println("DONE: stopped after 1 second");
  }
}
