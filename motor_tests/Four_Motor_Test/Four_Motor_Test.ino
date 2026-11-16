// Arduino Due, Programming Port. Test ONE raised wheel at a time.
// DIR: Due D7/D9/D11/D13 -> Cytron 1/3/5/7.
// EN: Due D6/D8/D10/D12 -> Cytron 2/4/6/8. GND -> G.
// Send 1, 2, 3 or 4 to select a motor, then f or r.
// Leave Cytron V disconnected. Battery powers Cytron, USB powers Due.
// Serial Monitor: 115200 baud. Send f or r for a 1-second test; s stops.
const int DIR_PINS[] = {7, 9, 11, 13};
const int EN_PINS[] = {6, 8, 10, 12};
int selected = 0;
const int TEST_PWM = 150;  // 0-255 duty setting, not measured wheel speed.
bool running = false;
unsigned long startedAt = 0;
unsigned long stoppedAt = 0;

void setup() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(EN_PINS[i], LOW);
    pinMode(EN_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i], OUTPUT);
    digitalWrite(DIR_PINS[i], LOW);
  }
  analogWriteResolution(8);
  Serial.begin(115200);
}

void loop() {
  if (running && millis() - startedAt >= 1000) {
    analogWrite(EN_PINS[selected], 0);
    running = false;
    stoppedAt = millis();
    Serial.println("Stopped. Wait 1 second before another test.");
  }
  if (!Serial.available()) return;
  char command = Serial.read();
  if (command >= '1' && command <= '4') {
    if (running) {
      Serial.println("Stop before selecting another motor.");
      return;
    }
    selected = command - '1';
    Serial.print("Selected M");
    Serial.println(selected + 1);
  } else if (command == 's') {
    analogWrite(EN_PINS[selected], 0);
    running = false;
    stoppedAt = millis();
    Serial.println("Stopped.");
  } else if (command == 'f' || command == 'r') {
    if (running || millis() - stoppedAt < 1000) {
      Serial.println("Wait until stopped for 1 second.");
      return;
    }
    digitalWrite(DIR_PINS[selected], command == 'f' ? LOW : HIGH);
    analogWrite(EN_PINS[selected], TEST_PWM);
    startedAt = millis();
    running = true;
    Serial.println("Running for 1 second.");
  }
}
