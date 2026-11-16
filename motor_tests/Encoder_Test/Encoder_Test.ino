// One encoder bench test, Arduino Due (Programming Port).
// Red -> Due 3.3V; GND wire -> Due GND; green SIG -> Due D2.
// Follow Lab 3's 3.3V supply, despite the encoder board's +5V label.
// Keep motor battery disconnected. Turn wheel gently by hand.
// Serial Monitor 115200 baud. Send z to zero the count.
const int ENCODER_PIN = 2;
volatile unsigned long pulseCount = 0;
unsigned long lastPrint = 0;

void countPulse() { pulseCount++; }

void setup() {
  // Disable motor outputs using the existing motor-test wiring.
  const int enablePins[] = {6, 8, 10, 12};
  for (int i = 0; i < 4; i++) {
    digitalWrite(enablePins[i], LOW);
    pinMode(enablePins[i], OUTPUT);
  }
  Serial.begin(115200);
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), countPulse, FALLING);
}

void loop() {
  if (Serial.available() && Serial.read() == 'z') {
    noInterrupts();
    pulseCount = 0;
    interrupts();
  }
  if (millis() - lastPrint >= 250) {
    lastPrint = millis();
    noInterrupts();
    unsigned long count = pulseCount;
    interrupts();
    Serial.print("Pulses: ");
    Serial.println(count);
  }
}
