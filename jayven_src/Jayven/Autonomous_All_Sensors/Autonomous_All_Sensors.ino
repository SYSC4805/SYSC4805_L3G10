// Arduino Due; AUTO START after 3 seconds. Serial 115200: s stops, f restarts.
// LEFT line OUT1 A1; RIGHT line OUT1 A0. LEFT IR D26; RIGHT IR D27.
// Ultrasonic TRIG D22, Echo D23. Sensors powered at 3.3V, common ground.
// No IMU/encoder speed control, Nano LED, battery monitoring or watchdog yet.
const int DIR_PINS[] = {7,9,11,13};
const int EN_PINS[] = {6,8,10,12};
const int FORWARD_LEVEL[] = {LOW,LOW,HIGH,HIGH};
const bool IS_LEFT[] = {true,true,false,false};
const int CRUISE_PWM=100, SLOW_PWM=75, RECOVERY_PWM=100;
const int TURN_PWM=255; // User-tested on floor: approximately 90 degrees in 1 second.
const int LINE_PINS[] = {A1,A0}; // LEFT, RIGHT
const int IR_PINS[] = {26,27};   // LEFT, RIGHT
const int BLACK[] = {3500,3400}; // LEFT, RIGHT; based on mounted floor/tape readings.
const int CLEAR[] = {3300,3100}; // LEFT floor ~3100; RIGHT floor ~2805.
const unsigned long START_DELAY_MS=3000, RUN_LIMIT_MS=290000;
const unsigned long PAUSE_MS=300, REVERSE_MS=400;
const unsigned long BOUNDARY_MIN_TURN_MS=1000, OBSTACLE_MIN_TURN_MS=1000;
const unsigned long MAX_TURN_MS=5000;
const unsigned long CLEAR_HOLD_MS=200;
const int FRONT_OFFSET_MM=0; // Distance from ultrasonic face.
const int SLOW_CLEARANCE_MM=300, RESUME_CLEARANCE_MM=350;
const int AVOID_DISTANCE_MM=200, TURN_CLEAR_DISTANCE_MM=250;
const int TRIG_PIN=22, ECHO_PIN=23;
const unsigned long PING_INTERVAL_US=60000, ECHO_TIMEOUT_US=35000;
volatile bool echoArmed=false, echoRisingSeen=false, echoComplete=false;
volatile unsigned long echoRiseUs=0, echoWidthUs=0;
bool pingActive=false, triggerHigh=false;
unsigned long pingStartedUs=0, lastPingUs=0;
bool rangeValid=false, slowMode=true;
const char *rangeReason="INIT";
unsigned int rangeCode=255;
int clearanceMm=0;
unsigned long lastRangeAt=0;

int samples[2][4]={}, sums[2]={}, lineAvg[2]={};
int sampleIndex=0, filled=0;
int lineState[2]={-1,-1}, lineCandidate[2]={-1,-1}, confirmations[2]={};
unsigned long lastLineAt=0;
bool irCandidate[2]={true,true}, irBlocked[2]={true,true};
unsigned long irSince[2]={};
enum State { STARTUP, READY, DRIVE, BOUNDARY_STOP, BACKUP, TURN_STOP,
             TURN, SENSOR_WAIT, STOPPED, COMPLETE, FAULT };
const char *NAMES[]={"STARTUP","READY","DRIVE","RECOVERY_STOP","BACKUP",
                    "TURN_STOP","TURN","SENSOR_WAIT","STOPPED","COMPLETE","FAULT"};
State state=STARTUP, resumeState=READY;
unsigned long stateSince=0, runSince=0, pausedElapsed=0, healthySince=0;
unsigned long clearSince=0, lastPrint=0;
bool runStarted=false, healthyTracking=false, clearTracking=false;
bool turnRight=true, boundaryTurn=false;

// Capture echo edges without pulseIn() blocking motor/sensor decisions.
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
void updateRange() { // Common range update interface; now ultrasonic, no I2C.
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
    rangeValid = mm >= 40 && mm <= 5500; // Conservative minimum usable range.
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
void motorsOff() {
  for (int i=0;i<4;i++) analogWrite(EN_PINS[i],0);
}
void driveSides(bool leftForward, bool rightForward, int pwm) {
  for (int i=0;i<4;i++) {
    bool forward=IS_LEFT[i] ? leftForward : rightForward;
    int direction=forward ? FORWARD_LEVEL[i] : !FORWARD_LEVEL[i];
    digitalWrite(DIR_PINS[i],direction);
    analogWrite(EN_PINS[i],pwm);
  }
}
void enter(State next) {
  motorsOff();
  state=next; stateSince=millis(); clearTracking=false;
}
void updateInputs() {
  unsigned long now=millis();
  for (int i=0;i<2;i++) {
    bool raw=digitalRead(IR_PINS[i])==LOW;
    if (raw!=irCandidate[i]) { irCandidate[i]=raw; irSince[i]=now; }
    if (now-irSince[i]>=(raw ? 20UL : 50UL)) irBlocked[i]=raw;
  }
  if (now-lastLineAt<5) return;
  lastLineAt=now;
  if (filled<4) filled++;
  for (int i=0;i<2;i++) {
    analogRead(LINE_PINS[i]); // Discard first conversion after changing channel.
    int raw=analogRead(LINE_PINS[i]);
    sums[i]-=samples[i][sampleIndex];
    samples[i][sampleIndex]=raw; sums[i]+=raw;
    lineAvg[i]=sums[i]/filled;
    if (filled<4) continue;
    int next=lineAvg[i]>BLACK[i] ? 1 : lineAvg[i]<CLEAR[i] ? 0 : -1;
    if (next<0) { confirmations[i]=0; lineCandidate[i]=-1; }
    else {
      if (next!=lineCandidate[i]) { lineCandidate[i]=next; confirmations[i]=0; }
      if (confirmations[i]<3) confirmations[i]++;
      if (confirmations[i]>=3) lineState[i]=next;
    }
  }
  sampleIndex=(sampleIndex+1)%4;
}
bool stableClear(bool clear, unsigned long now) {
  if (!clear) { clearTracking=false; return false; }
  if (!clearTracking) { clearSince=now; clearTracking=true; }
  return now-clearSince>=CLEAR_HOLD_MS;
}
void beginBoundary() {
  boundaryTurn=true;
  turnRight=(lineState[0]==1); // Left or both -> right. Right only -> left.
  enter(BOUNDARY_STOP);
}
void beginObstacle() {
  boundaryTurn=false;
  turnRight=irBlocked[0] || !irBlocked[1]; // Right-only IR -> left; otherwise right.
  enter(BOUNDARY_STOP); // Stop, back away, then turn; shared recovery sequence.
}
void setup() {
  for (int i=0;i<4;i++) {
    digitalWrite(EN_PINS[i],LOW); pinMode(EN_PINS[i],OUTPUT);
    digitalWrite(DIR_PINS[i],LOW); pinMode(DIR_PINS[i],OUTPUT);
  }
  analogWriteResolution(8); analogReadResolution(12);
  for (int i=0;i<2;i++) {
    pinMode(IR_PINS[i],INPUT);
    irCandidate[i]=digitalRead(IR_PINS[i])==LOW;
    irSince[i]=millis();
  }
  digitalWrite(TRIG_PIN,LOW); pinMode(TRIG_PIN,OUTPUT);
  pinMode(ECHO_PIN,INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN),echoChanged,CHANGE);
  Serial.begin(115200);
  stateSince=millis();
  // No wait for Serial: starts without a connected computer.
}
void controlRobot() {
  unsigned long now=millis();
  if (Serial.available()) {
    char c=Serial.read();
    if (c=='s') { enter(STOPPED); return; }
    if (c=='f' && (state==STOPPED || state==COMPLETE || state==FAULT)) {
      runStarted=false; enter(STARTUP);
    }
  }
  if (state==STOPPED || state==COMPLETE || state==FAULT) return;
  if (runStarted && now-runSince>=RUN_LIMIT_MS) { enter(COMPLETE); return; }
  bool healthy=rangeValid && now-lastRangeAt<=300;
  bool known=lineState[0]>=0 && lineState[1]>=0;
  bool tape=lineState[0]==1 || lineState[1]==1;
  bool obstacle=irBlocked[0] || irBlocked[1] || (healthy && clearanceMm<=AVOID_DISTANCE_MM);
  if (state==STARTUP) {
    if (now-stateSince>=START_DELAY_MS && known) {
      runSince=now; runStarted=true; enter(READY);
    }
    return;
  }
  if (!healthy && state!=SENSOR_WAIT) {
    resumeState=state; pausedElapsed=now-stateSince;
    healthyTracking=false; enter(SENSOR_WAIT);
  }
  if (state==SENSOR_WAIT) {
    if (!healthy) healthyTracking=false;
    else {
      if (!healthyTracking) { healthySince=now; healthyTracking=true; }
      if (now-healthySince>=500) {
        state=resumeState; stateSince=now-pausedElapsed; clearTracking=false;
      }
    }
    return; // Re-evaluate tape and IR before restoring motor output.
  }
  // Boundary outranks obstacle recovery, including an obstacle-triggered turn.
  // During boundary backup/turn, expected tape is handled by that recovery.
  if (tape && (state==READY || state==DRIVE ||
      (!boundaryTurn && (state==TURN_STOP || state==TURN)))) {
    beginBoundary(); return;
  }
  unsigned long elapsed=now-stateSince;
  switch (state) {
    case READY:
      if (obstacle) beginObstacle();
      else if (stableClear(!tape,now)) enter(DRIVE);
      break;
    case DRIVE:
      if (obstacle) beginObstacle();
      else driveSides(true,true,slowMode ? SLOW_PWM : CRUISE_PWM);
      break;
    case BOUNDARY_STOP:
      if (elapsed>=PAUSE_MS) enter(BACKUP);
      break;
    case BACKUP:
      // Front sensors cannot detect rear obstructions.
      if (elapsed>=REVERSE_MS) enter(TURN_STOP);
      else driveSides(false,false,RECOVERY_PWM);
      break;
    case TURN_STOP:
      if (elapsed>=PAUSE_MS) enter(TURN);
      break;
    case TURN:
      // Keep rotating through the original front detection until both are clear.
      // Boundary turn also needs minimum rotation to avoid returning to tape.
      if (elapsed>=MAX_TURN_MS) { enter(FAULT); break; }
      if (stableClear(!tape && !obstacle && clearanceMm>TURN_CLEAR_DISTANCE_MM,now) &&
          elapsed >= (boundaryTurn ? BOUNDARY_MIN_TURN_MS : OBSTACLE_MIN_TURN_MS)) {
        enter(READY);
      } else driveSides(turnRight,!turnRight,TURN_PWM);
      break;
    default: break;
  }
}
void printStatus() {
  unsigned long now=millis();
  if (now-lastPrint<250) return;
  char line[128];
  int cm=(rangeValid && now-lastRangeAt<=300) ? clearanceMm/10 : -1;
  int n=snprintf(line,sizeof(line),"L:%d/%d R:%d/%d IR:%d,%d cm:%d %s %s %s\n",
    lineAvg[0],lineState[0],lineAvg[1],lineState[1],irBlocked[0],irBlocked[1],cm,
    slowMode ? "SLOW" : "FAST",NAMES[state],rangeReason);
  // Skip logs rather than blocking sensor timing on a full serial buffer.
  if (n>0 && n<(int)sizeof(line) && Serial.availableForWrite()>=n) {
    Serial.write((const uint8_t*)line,n); lastPrint=now;
  }
}
void loop() {
  updateRange();
  updateInputs();
  controlRobot();
  if (!triggerHigh) printStatus();
}
