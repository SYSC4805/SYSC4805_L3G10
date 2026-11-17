// AJ Donald 101259149

// IR Motor stop test 
const int LEFT_IR_PIN = 26;
const int RIGHT_IR_PIN = 27;
const int DIR_PINS[] = {7, 9, 11, 13};
const int EN_PINS[] = {6, 8, 10, 12};
const int FORWARD_LEVEL[] = {LOW, LOW, HIGH, HIGH}; 
const int TEST_PWM = 100; 
bool running = false;
unsigned long stoppedAt = 0;
unsigned long lastPrint = 0;

void stopMotors() {
  for (int i = 0; i < 4; i++) analogWrite(EN_PINS[i], 0);
  running = false;
  stoppedAt = millis();
}
void setup() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(EN_PINS[i], LOW);
    pinMode(EN_PINS[i], OUTPUT);
    digitalWrite(DIR_PINS[i], LOW);
    pinMode(DIR_PINS[i], OUTPUT);
  }
  analogWriteResolution(8);
  pinMode(LEFT_IR_PIN, INPUT);
  pinMode(RIGHT_IR_PIN, INPUT);
  Serial.begin(115200);
}
void loop() {
  bool leftBlocked = digitalRead(LEFT_IR_PIN) == LOW;
  bool rightBlocked = digitalRead(RIGHT_IR_PIN) == LOW;
  bool obstacle = leftBlocked || rightBlocked;

  if (running && obstacle) {
    stopMotors();
    Serial.println("STOP: obstacle detected. Clear it, then send f to restart.");
  }
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 's') {
      stopMotors();
      Serial.println("STOP: manual command.");
    } else if (command == 'f') {
      if (obstacle) Serial.println("Start blocked: obstacle detected.");
      else if (running) Serial.println("Already moving.");
      else if (millis() - stoppedAt < 500) Serial.println("Wait half a second after stopping.");
      else {
        for (int i = 0; i < 4; i++) digitalWrite(DIR_PINS[i], FORWARD_LEVEL[i]);
        for (int i = 0; i < 4; i++) analogWrite(EN_PINS[i], TEST_PWM);
        running = true;
        Serial.println("FORWARD. Either IR sensor will stop the motors.");
      }
    }
  }
  unsigned long now = millis();
  if (now - lastPrint >= 250 && Serial.availableForWrite() >= 64) {
    lastPrint = now;
    Serial.print("Left: "); Serial.print(leftBlocked ? "OBSTACLE" : "clear");
    Serial.print(" | Right: "); Serial.print(rightBlocked ? "OBSTACLE" : "clear");
    Serial.println(running ? " | RUNNING" : " | STOPPED");
  }
}
