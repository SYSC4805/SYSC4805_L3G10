// automatic bench mode: starts when clear, loops without an overall timeout.
const int DIR_PINS[] = {7, 9, 11, 13};
const int EN_PINS[] = {6, 8, 10, 12};
const int FORWARD_LEVEL[] = {LOW, LOW, HIGH, HIGH};
const bool IS_LEFT[] = {true, true, false, false};
const int SENSOR_PIN = A0;
const int BLACK_THRESHOLD = 3400; // Enter tape above this ( confirm as you go)
const int CLEAR_THRESHOLD = 3100; // Return to floor below this
const int CONFIRM_SAMPLES = 3;    // Three consecutive averaged decisions
bool onTape = true;              // Startup waits for confirmed clear floor.
int confirmCount = 0;
const int SAMPLE_COUNT = 4;
const unsigned long SAMPLE_INTERVAL_MS = 5;
int samples[SAMPLE_COUNT] = {0};
int sampleIndex = 0;
int samplesFilled = 0;
unsigned long sampleSum = 0;
unsigned long lastSampleAt = 0;
int averagedReading = 0;
const int TEST_PWM = 100;
const unsigned long PAUSE_MS = 500;
const unsigned long REVERSE_MS = 400; 
const unsigned long TURN_MS = 900;    // Longer starting estimate.  calibrate for ~90 degrees.
enum State { IDLE, FORWARD, PAUSE_BEFORE_REVERSE, REVERSE, PAUSE_BEFORE_TURN, TURN_RIGHT };
State state = IDLE;
bool automaticEnabled = true;
bool clearTracking = false;
unsigned long clearSince = 0;
unsigned long stateStartedAt = 0;
unsigned long stoppedAt = 0;
unsigned long lastPrint = 0;

void motorsOff() {
  for (int i = 0; i < 4; i++) analogWrite(EN_PINS[i], 0);
}
void driveSides(bool leftForward, bool rightForward) {
  motorsOff();
  for (int i = 0; i < 4; i++) {
    bool forward = IS_LEFT[i] ? leftForward : rightForward;
    int direction = FORWARD_LEVEL[i];
    if (!forward) direction = (direction == LOW) ? HIGH : LOW;
    digitalWrite(DIR_PINS[i], direction);
  }
  for (int i = 0; i < 4; i++) analogWrite(EN_PINS[i], TEST_PWM);
}
void enterState(State next) {
  state = next;
  stateStartedAt = millis();
}
void finish(const char *message, bool manualPause = true) {
  motorsOff();
  if (manualPause) automaticEnabled = false;
  clearTracking = false;
  enterState(IDLE);
  stoppedAt = millis();
  Serial.println(message);
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
  unsigned long now = millis();
  if (now - lastSampleAt >= SAMPLE_INTERVAL_MS) {
    lastSampleAt = now;
    sampleSum -= samples[sampleIndex];
    samples[sampleIndex] = analogRead(SENSOR_PIN);
    sampleSum += samples[sampleIndex];
    sampleIndex = (sampleIndex + 1) % SAMPLE_COUNT;
    if (samplesFilled < SAMPLE_COUNT) samplesFilled++;
    averagedReading = sampleSum / samplesFilled;
    if (samplesFilled == SAMPLE_COUNT) {
      bool changeRequested = onTape ? averagedReading < CLEAR_THRESHOLD
                                    : averagedReading > BLACK_THRESHOLD;
      if (changeRequested) {
        confirmCount++;
        if (confirmCount >= CONFIRM_SAMPLES) {
          onTape = !onTape;
          confirmCount = 0;
        }
      } else confirmCount = 0;
    }
  }
  bool sensorReady = samplesFilled == SAMPLE_COUNT;
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 's') {
      finish("STOP: manual command.");
      return;
    }
    if (command == 'f' && state == IDLE) {
      automaticEnabled = true;
      clearTracking = false;
      Serial.println("AUTO enabled: waiting for clear sensor.");
    }
  }
  // Require 500 ms continuously clear before automatic start/resume.
  if (state == IDLE && automaticEnabled && sensorReady) {
    if (onTape) clearTracking = false;
    else {
      if (!clearTracking) {
        clearSince = millis();
        clearTracking = true;
      }
      if (millis() - clearSince >= PAUSE_MS && millis() - stoppedAt >= PAUSE_MS) {
        driveSides(true, true);
        enterState(FORWARD);
        clearTracking = false;
        Serial.println("AUTO FORWARD");
      }
    }
  }
  unsigned long elapsed = millis() - stateStartedAt;
  switch (state) {
    case FORWARD:
      if (onTape) {
        motorsOff();
        enterState(PAUSE_BEFORE_REVERSE);
        Serial.println("TAPE: stopped; preparing short reverse.");
      }
      break;
    case PAUSE_BEFORE_REVERSE:
      if (elapsed >= PAUSE_MS) {
        driveSides(false, false);
        enterState(REVERSE);
        Serial.println("REVERSE");
      }
      break;
    case REVERSE:
      // Expected tape while backing away does not repeatedly restart recovery.
      if (elapsed >= REVERSE_MS) {
        motorsOff();
        enterState(PAUSE_BEFORE_TURN);
        Serial.println("STOP: reverse finished.");
      }
      break;
    case PAUSE_BEFORE_TURN:
      if (elapsed >= PAUSE_MS) {
        if (onTape) finish("HOLD: tape still detected; auto-resume when clear.", false);
        else {
          driveSides(true, false);
          enterState(TURN_RIGHT);
          Serial.println("TURN RIGHT");
        }
      }
      break;
    case TURN_RIGHT:
      if (onTape) finish("HOLD: tape during turn; auto-resume when clear.", false);
      else if (elapsed >= TURN_MS) {
        motorsOff();
        enterState(IDLE);
        stoppedAt = millis();
        clearTracking = false;
        Serial.println("Turn finished: automatic forward resumes when clear.");
      }
      break;
    case IDLE:
      break;
  }
  now = millis();
  if (sensorReady && now - lastPrint >= 250 && Serial.availableForWrite() >= 60) {
    lastPrint = now;
    Serial.print("Average (4): "); Serial.print(averagedReading);
    Serial.print(onTape ? " | BLACK TAPE" : " | FLOOR");
    Serial.println(!automaticEnabled ? " | PAUSED" : (state == IDLE ? " | AUTO WAIT" : " | AUTO ACTIVE"));
  }
}
