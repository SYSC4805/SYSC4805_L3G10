// Arduino Due, Programming Port. One raised wheel on Cytron M1.
// Due D7 -> Cytron 1 (DIR); D6 -> Cytron 2 (EN); GND -> G.
// Leave Cytron V disconnected. Battery powers Cytron, USB powers Due.
// Serial Monitor: 115200 baud. Send f or r for a 1-second test; s stops.
const int DIR_PIN = 7;
const int EN_PIN = 6;
const int TEST_PWM = 150;  // 0-255 duty setting, not measured wheel speed.
bool running = false;
unsigned long startedAt = 0;
unsigned long stoppedAt = 0;

void setup() {
  digitalWrite(EN_PIN, LOW);
  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);
  analogWriteResolution(8);
  Serial.begin(115200);
}

void loop() {
  if (running && millis() - startedAt >= 1000) {
    analogWrite(EN_PIN, 0);
    running = false;
    stoppedAt = millis();
    Serial.println("Stopped. Wait 1 second before another test.");
  }
  if (!Serial.available()) return;
  char command = Serial.read();
  if (command == 's') {
    analogWrite(EN_PIN, 0);
    running = false;
    stoppedAt = millis();
    Serial.println("Stopped.");
  } else if (command == 'f' || command == 'r') {
    if (running || millis() - stoppedAt < 1000) {
      Serial.println("Wait until stopped for 1 second.");
      return;
    }
    digitalWrite(DIR_PIN, command == 'f' ? LOW : HIGH);
    analogWrite(EN_PIN, TEST_PWM);
    startedAt = millis();
    running = true;
    Serial.println("Running for 1 second.");
  }
}
