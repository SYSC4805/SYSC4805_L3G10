// AJ DONald 101259149

// IR Test to avoid objects with it on left side of the robot
const int DIR_PINS[] = {7,9,11,13};
const int EN_PINS[] = {6,8,10,12};
const int FORWARD_LEVEL[] = {LOW,LOW,HIGH,HIGH};
const bool IS_LEFT[] = {true,true,false,false};
const int IR_PIN = 26;
const int PWM_VALUE = 100;
// Adds detection delay
const unsigned long DETECT_MS = 20;
const unsigned long RELEASE_MS = 50;
bool rawObstacle = false;
bool candidateObstacle = false;
bool confirmedObstacle = true; // Wait for confirmed clear on startup.
unsigned long candidateSince = 0;
unsigned long rejectedLowPulses = 0;
unsigned long longestRejectedLowMs = 0;

const unsigned long PAUSE_MS = 500;
const unsigned long REVERSE_MS = 400;
const unsigned long TURN_MS = 900;
const unsigned long CLEAR_MS = 500;
enum State { DISABLED, WAIT_CLEAR, FORWARD, STOP_BEFORE_REVERSE,
             REVERSE, STOP_BEFORE_TURN, TURN_RIGHT, BLOCKED_HOLD };
State state = DISABLED;
unsigned long stateSince = 0, clearSince = 0, lastPrint = 0;
bool clearTracking = false;
unsigned long turnRemainingMs = TURN_MS;
const char *STATE_NAMES[] = {"DISABLED", "WAIT_CLEAR", "FORWARD", "PAUSE_BEFORE_REVERSE",
                             "REVERSE", "PAUSE_BEFORE_TURN", "TURN_RIGHT", "AUTO_HOLD"};

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
  Serial.println("FILTERED IR TEST: 20 ms detection; f starts, s stops.");
}
void loop() {
  updateIR();
  bool obstacle=confirmedObstacle;
  unsigned long now=millis();
  if (obstacle && state==FORWARD) {
    motorsOff(); enter(STOP_BEFORE_REVERSE,"OBSTACLE: stopped, preparing reverse.");
  } else if (obstacle && state==TURN_RIGHT) {
    motorsOff();
    unsigned long turnElapsed = millis() - stateSince;
    turnRemainingMs = turnElapsed >= turnRemainingMs ? 0 : turnRemainingMs - turnElapsed;
    enter(BLOCKED_HOLD,"AUTO HOLD: confirmed obstacle during turn; resumes after clear.");
  }
  if (Serial.available()) {
    char command=Serial.read();
    if (command=='s') {
      motorsOff(); enter(DISABLED,"DISABLED"); return;
    }
    if (command=='f' && state==DISABLED)
      enter(WAIT_CLEAR,"AUTO enabled: waiting for clear path.");
  }
  unsigned long elapsed=millis()-stateSince;
  switch(state) {
    case WAIT_CLEAR:
      if (obstacle) clearTracking=false;
      else {
        if (!clearTracking) { clearSince=now; clearTracking=true; }
        if (now-clearSince>=CLEAR_MS) {
          driveSides(true,true); enter(FORWARD,"FORWARD");
        }
      }
      break;
    case STOP_BEFORE_REVERSE:
      if (elapsed>=PAUSE_MS) {
        driveSides(false,false); enter(REVERSE,"REVERSE: 400 ms");
      }
      break;
    case REVERSE:
      // Expected front detection is allowed while backing away; no rear sensor.
      if (elapsed>=REVERSE_MS) {
        motorsOff();
        turnRemainingMs = TURN_MS;
        enter(STOP_BEFORE_TURN,"Stopped after reverse.");
      }
      break;
    case STOP_BEFORE_TURN:
      if (elapsed>=PAUSE_MS) {
        if (obstacle) enter(BLOCKED_HOLD,"AUTO HOLD: still blocked after reverse; waiting for clear.");
        else { driveSides(true,false); enter(TURN_RIGHT,"TURN RIGHT"); }
      }
      break;
    case TURN_RIGHT:
      if (elapsed>=turnRemainingMs) {
        motorsOff(); enter(WAIT_CLEAR,"Turn finished: resuming when clear.");
      }
      break;
    case BLOCKED_HOLD:
      if (obstacle) clearTracking=false;
      else {
        if (!clearTracking) { clearSince=now; clearTracking=true; }
        if (now-clearSince>=CLEAR_MS) {
          if (turnRemainingMs==0)
            enter(WAIT_CLEAR,"Turn time complete: preparing forward.");
          else {
            driveSides(true,false);
            enter(TURN_RIGHT,"RESUME RIGHT TURN: remaining time only.");
          }
        }
      }
      break;
    default: break;
  }
  now=millis();
  if (now-lastPrint>=250 && Serial.availableForWrite()>=63) {
    lastPrint=now;
    Serial.print("Raw:"); Serial.print(rawObstacle ? 0 : 1);
    Serial.print(" IR:"); Serial.print(obstacle ? "BLOCKED" : "clear");
    Serial.print(" S:"); Serial.print((int)state);
    Serial.print(" Glitches:"); Serial.print(rejectedLowPulses);
    Serial.print(" MaxMs:"); Serial.println(longestRejectedLowMs);
  }
}
