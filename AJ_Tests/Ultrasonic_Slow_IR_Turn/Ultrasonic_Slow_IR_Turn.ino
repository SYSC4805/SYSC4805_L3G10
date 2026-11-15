// AJ DOnald 101259149

// Ultrasonic + IR testing 
const int DIR_PINS[] = {7,9,11,13};
const int EN_PINS[] = {6,8,10,12};
const int FORWARD_LEVEL[] = {LOW,LOW,HIGH,HIGH};
const bool IS_LEFT[] = {true,true,false,false};
const int IR_PIN = 26;
const int PWM_VALUE = 100; 
const int CRUISE_PWM = 100;
const int SLOW_PWM = 75;
const int FRONT_OFFSET_MM = 0; // Distance measured from ultrasonic sensor face.
const int SLOW_CLEARANCE_MM = 200;
const int RESUME_CLEARANCE_MM = 250;
const int TRIG_PIN = 22;
const int ECHO_PIN = 23;
const unsigned long PING_INTERVAL_US = 60000;
const unsigned long ECHO_TIMEOUT_US = 35000;
volatile bool echoArmed = false, echoRisingSeen = false, echoComplete = false;
volatile unsigned long echoRiseUs = 0, echoWidthUs = 0;
bool pingActive = false, triggerHigh = false;
unsigned long pingStartedUs = 0;
unsigned long lastPingUs = 0;
bool rangeReady = false, rangeValid = false, slowMode = true;
const char *rangeReason = "INIT";
unsigned int rangeCode = 255;
int clearanceMm = 0;
unsigned long lastRangeAt = 0, lastRangePoll = 0;
const unsigned long DETECT_MS = 20;
const unsigned long RELEASE_MS = 50;
bool rawObstacle = false;
bool candidateObstacle = false;
bool confirmedObstacle = true; // Wait for confirmed clear on startup.
unsigned long candidateSince = 0;
unsigned long rejectedLowPulses = 0;
unsigned long longestRejectedLowMs = 0;

const unsigned long PAUSE_MS = 300;
const unsigned long MAX_TURN_MS = 5000; // Stop if IR never clears, pressing f retries.
const unsigned long TURN_CLEAR_MS = 200; // Require stable clear before ending turn.
const unsigned long CLEAR_MS = 500;
enum State { DISABLED, WAIT_CLEAR, FORWARD, STOP_BEFORE_TURN, TURN_RIGHT, SENSOR_WAIT };
State state = DISABLED;
State resumeState = WAIT_CLEAR;
unsigned long pausedElapsed = 0, healthySince = 0;
bool healthyTracking = false;
unsigned long stateSince = 0, clearSince = 0, lastPrint = 0;
bool clearTracking = false;
const char *STATE_NAMES[] = {"DISABLED", "WAIT_CLEAR", "FORWARD", "STOP", "TURN_RIGHT", "SENSOR_WAIT"};

void echoChanged() {
  if (!echoArmed) return;
  unsigned long now = micros();
  if (digitalRead(ECHO_PIN) == HIGH) {
    echoRiseUs = now;
    echoRisingSeen = true;
  } else if (echoRisingSeen) {
    echoWidthUs = now - echoRiseUs;
    echoComplete = true;
    echoArmed = false;
  }
}
void updateRange() { 
  unsigned long now = micros();
  if (triggerHigh && now-pingStartedUs >= 10) {
    digitalWrite(TRIG_PIN, LOW);
    triggerHigh = false;
  }
  bool complete;
  unsigned long width;
  noInterrupts();
  complete = echoComplete;
  width = echoWidthUs;
  if (complete) echoComplete = false;
  interrupts();
  if (pingActive && complete) {
    pingActive = false;
    unsigned long mm = width * 343UL / 2000UL;
    rangeValid = mm >= 40 && mm <= 5500; 
    rangeReason = rangeValid ? "OK" : "OUT_OF_RANGE";
    rangeCode = rangeValid ? 0 : 1;
    if (rangeValid) {
      lastRangeAt = millis();
      clearanceMm = (int)mm - FRONT_OFFSET_MM;
      if (clearanceMm < 0) clearanceMm = 0;
      if (clearanceMm <= SLOW_CLEARANCE_MM) slowMode = true;
      else if (clearanceMm > RESUME_CLEARANCE_MM) slowMode = false;
    }
  }
  if (pingActive && now-pingStartedUs >= ECHO_TIMEOUT_US) {
    noInterrupts(); echoArmed=false; echoComplete=false; interrupts();
    pingActive=false;
    rangeValid=false;
    rangeReason="NO_ECHO";
    rangeCode=2;
  }
  if (!pingActive && now-lastPingUs >= PING_INTERVAL_US) {
    lastPingUs=now;
    if (digitalRead(ECHO_PIN)==HIGH) {
      rangeValid=false; rangeReason="ECHO_STUCK_HIGH"; rangeCode=3; return;
    }
    noInterrupts();
    echoComplete=false; echoRisingSeen=false; echoArmed=true;
    interrupts();
    pingStartedUs=micros();
    pingActive=true;
    triggerHigh=true;
    digitalWrite(TRIG_PIN,HIGH);
  }
}
void forwardDrive() {
  driveSides(true, true);
  for (int i=0;i<4;i++) analogWrite(EN_PINS[i], slowMode ? SLOW_PWM : CRUISE_PWM);
}
void updateIR() {
  unsigned long now = millis();
  bool nextRaw = digitalRead(IR_PIN) == LOW;
  if (nextRaw != candidateObstacle) {
    if (candidateObstacle && !confirmedObstacle) {
      unsigned long width = now - candidateSince;
      rejectedLowPulses++;
      if (width > longestRejectedLowMs) longestRejectedLowMs = width;
    }
    candidateObstacle = nextRaw;
    candidateSince = now;
  }
  rawObstacle = nextRaw;
  unsigned long required = candidateObstacle ? DETECT_MS : RELEASE_MS;
  if (now - candidateSince >= required) confirmedObstacle = candidateObstacle;
}
void motorsOff() {
  for (int i=0;i<4;i++) analogWrite(EN_PINS[i],0);
}
void driveSides(bool leftForward, bool rightForward) {
  motorsOff();
  for (int i=0;i<4;i++) {
    bool forward = IS_LEFT[i] ? leftForward : rightForward;
    int direction = FORWARD_LEVEL[i];
    if (!forward) direction = direction == LOW ? HIGH : LOW;
    digitalWrite(DIR_PINS[i],direction);
  }
  for (int i=0;i<4;i++) analogWrite(EN_PINS[i],PWM_VALUE);
}
void enter(State next, const char *message) {
  state=next;
  stateSince=millis();
  clearTracking=false;
  Serial.println(message);
}
void setup() {
  for (int i=0;i<4;i++) {
    digitalWrite(EN_PINS[i],LOW); pinMode(EN_PINS[i],OUTPUT);
    digitalWrite(DIR_PINS[i],LOW); pinMode(DIR_PINS[i],OUTPUT);
  }
  analogWriteResolution(8);
  pinMode(IR_PIN,INPUT);
  candidateObstacle = digitalRead(IR_PIN) == LOW;
  candidateSince = millis();
  Serial.begin(115200);
  Serial.println("ULTRASONIC + LEFT IR: f enables; s stops. No echo pauses motion.");
  digitalWrite(TRIG_PIN,LOW); pinMode(TRIG_PIN,OUTPUT);
  pinMode(ECHO_PIN,INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN),echoChanged,CHANGE);
  rangeReady=true;

}
void loop() {
  updateIR();
  updateRange();
  unsigned long now = millis();
  bool healthy = rangeReady && rangeValid && now-lastRangeAt <= 300;

  if (Serial.available()) {
    char command = Serial.read();
    if (command == 's') {
      motorsOff(); enter(DISABLED, "DISABLED"); return;
    }
    if (command == 'f' && state == DISABLED)
      enter(WAIT_CLEAR, "AUTO enabled: waiting for IR clear.");
  }

  if (!healthy && state != DISABLED && state != SENSOR_WAIT) {
    motorsOff();
    resumeState = state;
    pausedElapsed = millis()-stateSince;
    healthyTracking = false;
    enter(SENSOR_WAIT, "Invalid ultrasonic: paused until readings recover.");
  }

  switch (state) {
    case SENSOR_WAIT:
      if (!healthy) healthyTracking = false;
      else {
        if (!healthyTracking) { healthySince=now; healthyTracking=true; }
        if (now-healthySince >= 500) {
          state=resumeState;
          stateSince=millis()-pausedElapsed;
          clearTracking=false;
          // Forward is applied next loop only after checking IR.
          if (state==TURN_RIGHT) driveSides(true,false);
          Serial.println("Range recovered.");
        }
      }
      break;
    case WAIT_CLEAR:
      motorsOff();
      if (confirmedObstacle) clearTracking=false;
      else {
        if (!clearTracking) { clearSince=now; clearTracking=true; }
        if (now-clearSince>=CLEAR_MS) {
          forwardDrive(); enter(FORWARD, "FORWARD");
        }
      }
      break;
    case FORWARD:
      if (confirmedObstacle) {
        motorsOff(); enter(STOP_BEFORE_TURN, "IR detected: STOP, then spin right.");
      } else {
        forwardDrive();
      }
      break;
    case STOP_BEFORE_TURN:
      if (millis()-stateSince>=PAUSE_MS) {
        driveSides(true,false); enter(TURN_RIGHT, "SPIN RIGHT: until IR stays clear");
      }
      break;
    case TURN_RIGHT:
      // Front IR is expected to remain blocked as we rotate away.
      if (confirmedObstacle) clearTracking=false;
      else {
        if (!clearTracking) { clearSince=now; clearTracking=true; }
        if (now-clearSince>=TURN_CLEAR_MS) {
          motorsOff(); enter(WAIT_CLEAR, "IR clear: turn finished, preparing forward.");
          break;
        }
      }
      if (millis()-stateSince>=MAX_TURN_MS) {
        motorsOff(); enter(DISABLED, "TURN LIMIT: IR did not clear. Check surroundings/sensor; f retries.");
      }
      break;
    default: break;
  }
  if (now-lastPrint>=250 && Serial.availableForWrite()>=63) {
    lastPrint=now;
    Serial.print("Distance:");
    if (healthy) Serial.print(clearanceMm/10.0f,1); else Serial.print("--");
    Serial.print("cm IR:"); Serial.print(confirmedObstacle ? "BLOCKED" : "clear");
    Serial.print(slowMode ? " SLOW " : " CRUISE ");
    Serial.print(STATE_NAMES[state]);
    Serial.print(" Range:"); Serial.println(healthy ? "OK" : rangeReason);
  }
}
