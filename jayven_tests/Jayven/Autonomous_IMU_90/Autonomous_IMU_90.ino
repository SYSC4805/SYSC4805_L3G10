#include <Wire.h>
#include <math.h>
// Arduino Due; AUTO START after gyro calibration and 3-second countdown. Serial 115200: s stops, f restarts.
// LEFT line OUT1 A1; RIGHT line OUT1 A0. LEFT IR D26; RIGHT IR D27.
// Ultrasonic TRIG D22, Echo D23. Sensors powered at 3.3V, common ground.
// MinIMU-9 v5: VIN 3.3V, SDA D20, SCL D21, GND common. Mount board flat.
// Gyro-relative 90-degree turns; Nano LED UART output; no encoder speed control or watchdog yet.
const int DIR_PINS[] = {7,9,11,13};
const int EN_PINS[] = {6,8,10,12};
const int FORWARD_LEVEL[] = {LOW,LOW,HIGH,HIGH};
const bool IS_LEFT[] = {true,true,false,false};
const int CRUISE_PWM=100, SLOW_PWM=75, RECOVERY_PWM=100;
const int TURN_PWM=255, TURN_APPROACH_PWM=180; // Reduce power for final 20 degrees.
const float TURN_TARGET_DEG=90.0f;
const int LINE_PINS[] = {A1,A0}; // LEFT, RIGHT
const int IR_PINS[] = {26,27};   // LEFT, RIGHT
const int BLACK[] = {3500,3400}; // LEFT, RIGHT; based on mounted floor/tape readings.
const int CLEAR[] = {3300,3100}; // LEFT floor ~3100; RIGHT floor ~2805.
const unsigned long START_DELAY_MS=3000, RUN_LIMIT_MS=290000;
const unsigned long PAUSE_MS=300, REVERSE_MS=400;
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
             TURN, SENSOR_WAIT, STOPPED, COMPLETE, FAULT, TURN_CHECK };
const char *NAMES[]={"STARTUP","READY","DRIVE","RECOVERY_STOP","BACKUP",
                    "TURN_STOP","TURN","SENSOR_WAIT","STOPPED","COMPLETE","FAULT","TURN_CHECK"};
State state=STARTUP, resumeState=READY;
unsigned long stateSince=0, runSince=0, pausedElapsed=0, healthySince=0;
unsigned long clearSince=0, lastPrint=0;
bool runStarted=false, healthyTracking=false, clearTracking=false;
bool turnRight=true, boundaryTurn=false;

// LSM6DS33 direct checked I2C transactions: no extra library needed.
uint8_t imuAddress=0;
bool imuPresent=false, imuCalibrated=false, imuFault=false;
const char *imuReason="INIT";
unsigned long imuBootAt=0, imuPollUs=0, imuSampleUs=0;
unsigned int calibrationCount=0;
float calibrationSum=0, calibrationSquares=0, gyroBias=0;
float yawDeg=0, turnStartYaw=0, previousRate=0;
unsigned int quarterTurns=0;
const unsigned int CALIBRATION_SAMPLES=300;

bool readImu(uint8_t reg, uint8_t *data, uint8_t count) {
  Wire.beginTransmission(imuAddress); Wire.write(reg);
  if (Wire.endTransmission()!=0) return false;
  uint8_t got=Wire.requestFrom(imuAddress,count);
  if (got!=count || Wire.available()<count) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (uint8_t i=0;i<count;i++) data[i]=Wire.read();
  return true;
}
bool writeImu(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(imuAddress); Wire.write(reg); Wire.write(value);
  return Wire.endTransmission()==0;
}
void failImu(const char *reason) {
  imuFault=true; imuReason=reason;
  enter(FAULT); // No timed-turn fallback with untrusted gyro data.
}
void beginImu() {
  motorsOff();
  imuPresent=false; imuCalibrated=false; imuFault=false;
  calibrationCount=0; calibrationSum=calibrationSquares=0;
  yawDeg=0; previousRate=0; imuSampleUs=0;
  Wire.begin(); Wire.setClock(100000);
  uint8_t id=0;
  for (uint8_t address=0x6A;address<=0x6B;address++) {
    imuAddress=address;
    if (readImu(0x0F,&id,1) && id==0x69) { imuPresent=true; break; }
  }
  if (!imuPresent) { failImu("NOT_FOUND"); return; }
  // BDU + auto-increment; accel104Hz +/-2g; gyro416Hz +/-500dps (0.0175dps/LSB).
  if (!writeImu(0x12,0x44) || !writeImu(0x10,0x40) || !writeImu(0x11,0x64)) {
    failImu("CONFIG_ERROR"); return;
  }
  uint8_t check=0;
  if (!readImu(0x11,&check,1) || check!=0x64) { failImu("CONFIG_ERROR"); return; }
  imuBootAt=millis(); imuPollUs=micros(); imuReason="KEEP_STILL";
}
void updateImu() {
  if (!imuPresent || imuFault) return;
  unsigned long nowUs=micros();
  if (nowUs-imuPollUs<5000) return;
  imuPollUs=nowUs;
  if (millis()-imuBootAt<500) return; // Let sensor settle with motors off.
  uint8_t status=0, data[6];
  if (!readImu(0x1E,&status,1)) { failImu("I2C_ERROR"); return; }
  if (!(status & 0x02)) {
    if ((imuSampleUs && nowUs-imuSampleUs>50000) ||
        (!imuCalibrated && millis()-imuBootAt>5000)) failImu("STALE");
    return;
  }
  if (!readImu(0x22,data,6)) { failImu("I2C_ERROR"); return; }
  int16_t rawZ=(int16_t)((uint16_t)data[5]<<8 | data[4]);
  float rate=rawZ*0.0175f;
  unsigned long sampled=micros();
  if (!imuCalibrated) {
    // Also check the board is roughly flat; yaw uses its Z axis.
    if (!readImu(0x28,data,6)) { failImu("I2C_ERROR"); return; }
    int16_t ax=(int16_t)((uint16_t)data[1]<<8 | data[0]);
    int16_t ay=(int16_t)((uint16_t)data[3]<<8 | data[2]);
    int16_t az=(int16_t)((uint16_t)data[5]<<8 | data[4]);
    if (abs(ax)>6000 || abs(ay)>6000 || abs(az)<12000 || abs(az)>20000) {
      failImu("MOUNT_FLAT"); return;
    }
    if (fabsf(rate)>10) { failImu("MOVED_DURING_CAL"); return; }
    calibrationSum+=rate; calibrationSquares+=rate*rate; calibrationCount++;
    imuSampleUs=sampled;
    if (calibrationCount>=CALIBRATION_SAMPLES) {
      gyroBias=calibrationSum/calibrationCount;
      float variance=calibrationSquares/calibrationCount-gyroBias*gyroBias;
      if (variance>1.0f || fabsf(gyroBias)>5.0f) { failImu("MOVED_DURING_CAL"); return; }
      imuCalibrated=true; imuReason="OK"; yawDeg=0; previousRate=rate-gyroBias;
      stateSince=millis(); // Three-second countdown starts AFTER calibration.
    }
    return;
  }
  unsigned long dt=sampled-imuSampleUs;
  if (dt>50000) { failImu("SAMPLE_GAP"); return; }
  imuSampleUs=sampled;
  if (abs(rawZ)>28000) { failImu("GYRO_SATURATED"); return; }
  rate-=gyroBias;
  yawDeg+=0.5f*(rate+previousRate)*(dt/1000000.0f);
  previousRate=rate;
}
float turnDegrees() { return fabsf(yawDeg-turnStartYaw); }

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
  if (next==TURN) { turnStartYaw=yawDeg; quarterTurns++; }
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
  boundaryTurn=true; quarterTurns=0;
  turnRight=(lineState[0]==1); // Left or both -> right. Right only -> left.
  enter(BOUNDARY_STOP);
}
void beginObstacle() {
  boundaryTurn=false; quarterTurns=0;
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
  Serial1.begin(9600); // D18 TX -> shifted Nano D0 RX.
  beginImu();
  stateSince=millis();
  // No wait for Serial: starts without a connected computer.
}
void controlRobot() {
  unsigned long now=millis();
  if (Serial.available()) {
    char c=Serial.read();
    if (c=='s') { enter(STOPPED); return; }
    if (c=='f' && (state==STOPPED || state==COMPLETE || state==FAULT)) {
      runStarted=false; enter(STARTUP); beginImu();
    }
  }
  if (state==STOPPED || state==COMPLETE || state==FAULT) return;
  if (runStarted && now-runSince>=RUN_LIMIT_MS) { enter(COMPLETE); return; }
  bool healthy=rangeValid && now-lastRangeAt<=300;
  bool known=lineState[0]>=0 && lineState[1]>=0;
  bool tape=lineState[0]==1 || lineState[1]==1;
  bool obstacle=irBlocked[0] || irBlocked[1] || (healthy && clearanceMm<=AVOID_DISTANCE_MM);
  if (state==STARTUP) {
    if (imuCalibrated && now-stateSince>=START_DELAY_MS && known) {
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
      (!boundaryTurn && (state==TURN_STOP || state==TURN || state==TURN_CHECK)))) {
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
      if (elapsed>=MAX_TURN_MS) { enter(FAULT); imuReason="TURN_TIMEOUT"; break; }
      // Angle alone ends this 90-degree segment; then check path while stopped.
      if (turnDegrees()>=TURN_TARGET_DEG) enter(TURN_CHECK);
      else driveSides(turnRight,!turnRight,
             turnDegrees()>=70.0f ? TURN_APPROACH_PWM : TURN_PWM);
      break;
    case TURN_CHECK:
      // Let chassis settle; telemetry includes any coast beyond the target.
      if (elapsed<500) break;
      if (!tape && !obstacle && clearanceMm>TURN_CLEAR_DISTANCE_MM) {
        if (stableClear(true,now)) enter(READY);
      } else {
        clearTracking=false;
        if (quarterTurns>=3) { enter(FAULT); imuReason="PATH_BLOCKED"; }
        else enter(TURN_STOP); // Another measured 90 degrees, at most 3 segments.
      }
      break;
    default: break;
  }
}
void printStatus() {
  unsigned long now=millis();
  char line[128];
  int cm=(rangeValid && now-lastRangeAt<=300) ? clearanceMm/10 : -1;
  int n=snprintf(line,sizeof(line),"L:%d/%d R:%d/%d IR:%d,%d cm:%d %s %s %s deg:%d IMU:%s\n",
    lineAvg[0],lineState[0],lineAvg[1],lineState[1],irBlocked[0],irBlocked[1],cm,
    slowMode ? "SLOW" : "FAST",NAMES[state],rangeReason,(int)turnDegrees(),imuReason);
  // Write a bounded chunk per loop so diagnostics never block gyro updates.
  static char pending[128];
  static int sent=0, length=0;
  if (now-lastPrint>=250 && sent>=length && n>0 && n<(int)sizeof(line)) {
    memcpy(pending,line,n); length=n; sent=0; lastPrint=now;
  }
  int room=Serial.availableForWrite();
  int count=min(room,length-sent);
  if (count>0) { Serial.write((const uint8_t*)pending+sent,count); sent+=count; }

}
// One-way state heartbeat; receiver never commands the motors.
void sendLedState() {
  static int previous=-1;
  static unsigned long lastSent=0;
  byte code=0;
  switch(state) {
    case DRIVE: code=slowMode ? 2 : 1; break;
    case BACKUP: code=4; break;
    case TURN: code=5; break;
    case COMPLETE: code=6; break;
    case FAULT: case SENSOR_WAIT: code=7; break;
    case STARTUP: code=0; break;
    default: code=3; break; // Pauses, stopped and turn-clearance checks.
  }
  if ((code!=previous || millis()-lastSent>=200) && Serial1.availableForWrite()>=3) {
    Serial1.write((uint8_t)0xA5);
    Serial1.write(code);
    Serial1.write((uint8_t)(code^0xFF));
    previous=code;lastSent=millis();
  }
}
void loop() {
  updateRange();
  updateInputs();
  if (!triggerHigh) updateImu();
  controlRobot();
  if (!triggerHigh) { printStatus(); sendLedState(); }
}
