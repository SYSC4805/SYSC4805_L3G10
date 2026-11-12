// Arduino Due (Programming Port). Test one encoder-equipped wheel, raised.
// Its motor -> Cytron M1 A/B. Due D7 -> Cytron 1; D6 -> Cytron 2.
// Encoder SIG -> Due D2; red -> Due 3.3V; GND -> Due GND.
// Due GND -> Cytron G. Leave Cytron V disconnected.
// Serial 115200: f / b run for 3 seconds, s stops early.
const int DIR_PIN = 7;
const int EN_PIN = 6;
const int ENCODER_PIN = 2;
const float PULSES_PER_REV = 11.0f; // Left encoder 1: 11. Right encoder 2: 12.
const float WHEEL_DIAMETER_CM = 6.5f;
const float CM_PER_PULSE = 3.14159265f * WHEEL_DIAMETER_CM / PULSES_PER_REV;
const int TEST_PWM = 150; // Duty setting, NOT speed in cm/s.
const unsigned long RUN_MS = 3000;
volatile unsigned long pulses = 0;
bool running = false;
unsigned long startedAt = 0, stoppedAt = 0;
unsigned long lastSampleAt = 0, lastPulses = 0;

void countPulse() { pulses++; }
unsigned long readPulses() {
  noInterrupts();
  unsigned long count = pulses;
  interrupts();
  return count;
}
void stopMotor() {
  analogWrite(EN_PIN, 0);
  running = false;
  stoppedAt = millis();
  Serial.println("Motor disabled. Wait 1 second before another run.");
}
void setup() {
  const int enables[] = {6, 8, 10, 12};
  for (int i = 0; i < 4; i++) {
    digitalWrite(enables[i], LOW);
    pinMode(enables[i], OUTPUT);
  }
  digitalWrite(DIR_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT);
  analogWriteResolution(8);
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), countPulse, FALLING);
  Serial.begin(115200);
  lastSampleAt = millis();
}
void loop() {
  unsigned long now = millis();
  if (running && now - startedAt >= RUN_MS) stopMotor();
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 's') stopMotor();
    else if (command == 'f' || command == 'b') {
      if (running || now - stoppedAt < 1000) {
        Serial.println("Wait until stopped for 1 second.");
      } else {
        noInterrupts();
        pulses = 0;
        interrupts();
        lastPulses = 0;
        lastSampleAt = now;
        digitalWrite(DIR_PIN, command == 'f' ? LOW : HIGH);
        startedAt = now;
        analogWrite(EN_PIN, TEST_PWM);
        running = true;
        Serial.println("Running for 3 seconds. Counts reset.");
      }
    }
  }
  // One-second windows reduce quantization from the low pulse resolution.
  now = millis();
  unsigned long elapsed = now - lastSampleAt;
  if (elapsed >= 1000) {
    unsigned long count = readPulses();
    float speed = (count - lastPulses) * CM_PER_PULSE * 1000.0f / elapsed;
    Serial.print("Pulses: "); Serial.print(count);
    Serial.print(" | Wheel travel: "); Serial.print(count * CM_PER_PULSE, 1);
    Serial.print(" cm | Speed: "); Serial.print(speed, 1);
    Serial.println(" cm/s");
    lastPulses = count;
    lastSampleAt = now;
  }
}
