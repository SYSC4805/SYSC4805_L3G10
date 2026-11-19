// Due + MinIMU-9 v5. Serial 115200: f calibrate/start, s stop.
// Keep still during calibration. Encoders retain provisional calibration.
// IMU errors latch timed fallback until restart. Other stops remain active.
#include <Wire.h>
#include <math.h>

const int DIR_PINS[] = {7,9,11,13};
const int EN_PINS[] = {6,8,10,12};
const int FORWARD_LEVEL[] = {LOW,LOW,HIGH,HIGH};
const bool IS_LEFT[] = {true,true,false,false};
const int CRUISE_PWM=150, APPROACH_PWM=110, RECOVERY_PWM=125;
const int TURN_PWM=200;
int motorCommand[4]={0,0,0,0};
const unsigned long MAX_TURN_MS=5000;
const unsigned long WATCHDOG_MS=750;
void watchdogSetup(){watchdogEnable(WATCHDOG_MS);}
uint32_t bootResetType=0;
unsigned long maxLoopUs=0;
const int LINE_PINS[] = {A1,A0};
const int IR_PINS[] = {26,27};
const int BLACK[] = {3500,3400};
const int CLEAR[] = {3300,3100};
const unsigned long START_DELAY_MS=3000, RUN_LIMIT_MS=290000;
const unsigned long PAUSE_MS=300, REVERSE_MS=400;
const unsigned long CLEAR_HOLD_MS=200;
const int FRONT_OFFSET_MM=40;
const int AVOID_DISTANCE_MM=200, TURN_CLEAR_DISTANCE_MM=250;
const int TRIG_PIN=22, ECHO_PIN=23;
const unsigned long PING_INTERVAL_US=60000, ECHO_TIMEOUT_US=35000;
volatile bool echoArmed=false, echoRisingSeen=false, echoComplete=false;
volatile unsigned long echoRiseUs=0, echoWidthUs=0;
bool pingActive=false, triggerHigh=false;
unsigned long pingStartedUs=0, lastPingUs=0;
bool rangeValid=false;
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
void enter(State next);
const char *NAMES[]={"STARTUP","READY","DRIVE","RECOVERY_STOP","BACKUP",
                    "TURN_STOP","TURN","SENSOR_WAIT","STOPPED","COMPLETE","FAULT","TURN_CHECK"};
State state=STOPPED, resumeState=READY;
unsigned long stateSince=0, runSince=0, pausedElapsed=0, healthySince=0;
unsigned long clearSince=0, lastPrint=0;
bool runStarted=false, healthyTracking=false, clearTracking=false;
bool turnRight=true, boundaryTurn=false;

const int ENCODER_PINS[2]={2,3};
const int LED_PINS[3]={30,31,32};
const float CM_PER_PULSE=3.14159265f*6.5f/10.0f;
const float CRUISE_CM_S=25.0f, APPROACH_CM_S=15.0f;
const int APPROACH_PWM_MAX=125;
const int APPROACH_MM=350, CRUISE_RESUME_MM=400;
bool approachSlow=false, feedbackWasSlow=false;
bool nextObstacleRight=false;
const float SPEED_DEADBAND=2.0f;

const bool ENABLE_OVERSPEED_STOP=false;
const int DRIVE_PWM_MIN=40, DRIVE_PWM_MAX=170;
const unsigned long SPEED_WINDOW_MS=1000, DRIVE_CONTROL_MS=1000;
const unsigned int NO_PULSE_WINDOWS=2;
volatile unsigned long encoderCount[2]={0,0};
unsigned long encoderPrevious[2]={0,0}, encoderSampleAt=0;
unsigned long drivePrevious[2]={0,0}, driveSampleAt=0;
float wheelSpeed[2]={0,0};
int drivePwm[2]={CRUISE_PWM,CRUISE_PWM};
unsigned int missingWindows[2]={0,0}, overspeedWindows[2]={0,0};
const char *controlReason="OK";
uint8_t imuAddress=0;
bool imuPresent=false, imuCalibrated=false, imuFault=false;
const char *imuReason="INIT";
unsigned long imuBootAt=0, imuPollUs=0, imuSampleUs=0;
unsigned int calibrationCount=0;
float calibrationSum=0, calibrationSquares=0, gyroBias=0;
float yawDeg=0, turnStartYaw=0, previousRate=0;

const unsigned int CALIBRATION_SAMPLES=300;

bool timedFallback=false, turnMotorActive=false;
float timedDegrees=0;
unsigned long timedAt=0;
void updateTimedHeading(){
  unsigned long now=millis();
  if(turnMotorActive) timedDegrees+=(now-timedAt)*90.0f/950.0f;
  timedAt=now;
}

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
  updateTimedHeading();
  imuFault=true; imuReason=reason; timedFallback=true;
}
void beginImu() {
  motorsOff();
  imuPresent=false; imuCalibrated=false; imuFault=false;
  timedFallback=false; timedDegrees=0; timedAt=millis(); turnMotorActive=false;
  calibrationCount=0; calibrationSum=calibrationSquares=0;
  yawDeg=0; previousRate=0; imuSampleUs=0;
  Wire.begin(); Wire.setClock(100000);
  uint8_t id=0;
  for (uint8_t address=0x6A;address<=0x6B;address++) {
    imuAddress=address;
    if (readImu(0x0F,&id,1) && id==0x69) { imuPresent=true; break; }
  }
  if (!imuPresent) { failImu("NOT_FOUND"); return; }

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
  if (millis()-imuBootAt<500) return;
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
      stateSince=millis();
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
float turnDegrees() { return timedFallback ? timedDegrees : fabsf(yawDeg-turnStartYaw); }

void leftEncoderPulse(){encoderCount[0]++;}
void rightEncoderPulse(){encoderCount[1]++;}
void encoderSnapshot(unsigned long &left,unsigned long &right){
  noInterrupts(); left=encoderCount[0]; right=encoderCount[1]; interrupts();
}
void updateEncoders(){
  unsigned long now=millis(),dt=now-encoderSampleAt;
  if(dt<SPEED_WINDOW_MS)return;
  unsigned long count[2];encoderSnapshot(count[0],count[1]);
  for(int i=0;i<2;i++){
    wheelSpeed[i]=(count[i]-encoderPrevious[i])*CM_PER_PULSE*1000.0f/dt;
    encoderPrevious[i]=count[i];
  }
  encoderSampleAt=now;
}
void resetDriveFeedback(){
  encoderSnapshot(drivePrevious[0],drivePrevious[1]);
  driveSampleAt=millis();feedbackWasSlow=approachSlow;
  for(int i=0;i<2;i++){
    drivePwm[i]=approachSlow?APPROACH_PWM:CRUISE_PWM;
    missingWindows[i]=overspeedWindows[i]=0;
  }
}

int adjustedPwm(int current,float target,float measured){
  float error=target-measured;
  if(fabsf(error)<=SPEED_DEADBAND)return current;
  int step=(int)roundf(error*0.5f);
  step=constrain(step,-12,8);
  return constrain(current+step,DRIVE_PWM_MIN,DRIVE_PWM_MAX);
}
void driveWithSpeedFeedback(){
  if(approachSlow!=feedbackWasSlow)resetDriveFeedback();
  unsigned long now=millis();
  unsigned long dt=now-driveSampleAt;
  if(dt>=DRIVE_CONTROL_MS){
    unsigned long count[2];encoderSnapshot(count[0],count[1]);
    for(int i=0;i<2;i++){
      unsigned long delta=count[i]-drivePrevious[i];
      float measured=delta*CM_PER_PULSE*1000.0f/dt;
      drivePrevious[i]=count[i];
      missingWindows[i]=delta==0?missingWindows[i]+1:0;
      overspeedWindows[i]=(ENABLE_OVERSPEED_STOP && measured>30.0f)
        ? min(overspeedWindows[i]+1,2U) : 0;

      if(missingWindows[i]>=NO_PULSE_WINDOWS || (ENABLE_OVERSPEED_STOP && overspeedWindows[i]>=2)){
        controlReason=missingWindows[i]>=NO_PULSE_WINDOWS?"ENCODER_NO_PULSES":"ENCODER_OVERSPEED";
        enter(FAULT);return;
      }
      if(delta>0)drivePwm[i]=adjustedPwm(drivePwm[i],approachSlow?APPROACH_CM_S:CRUISE_CM_S,measured);
      if(approachSlow)drivePwm[i]=min(drivePwm[i],APPROACH_PWM_MAX);
    }
    driveSampleAt=now;
  }
  for(int i=0;i<4;i++){
    digitalWrite(DIR_PINS[i],FORWARD_LEVEL[i]);
    motorCommand[i]=drivePwm[IS_LEFT[i]?0:1];
    analogWrite(EN_PINS[i],motorCommand[i]);
  }
}

unsigned int turnSegments=0;

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
  updateTimedHeading(); turnMotorActive=false;

  for (int i=0;i<4;i++){motorCommand[i]=0;analogWrite(EN_PINS[i],0);}
}
void driveSides(bool leftForward, bool rightForward, int pwm) {
  updateTimedHeading(); turnMotorActive=leftForward!=rightForward && pwm>0;

  for (int i=0;i<4;i++) {
    bool forward=IS_LEFT[i] ? leftForward : rightForward;
    int direction=forward ? FORWARD_LEVEL[i] : !FORWARD_LEVEL[i];
    digitalWrite(DIR_PINS[i],direction);
    motorCommand[i]=pwm;analogWrite(EN_PINS[i],pwm);
  }
}
void enter(State next) {
  motorsOff();
  state=next; stateSince=millis(); clearTracking=false;
  if (next==TURN) {turnSegments++;turnStartYaw=yawDeg;timedDegrees=0;timedAt=millis();}
  if (next==DRIVE) resetDriveFeedback();
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
    analogRead(LINE_PINS[i]);
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
  boundaryTurn=true; turnSegments=0;
  turnRight=(lineState[0]==1);
  enter(BOUNDARY_STOP);
}
void beginObstacle() {
  boundaryTurn=false; turnSegments=0;
  turnRight=nextObstacleRight;nextObstacleRight=!nextObstacleRight;
  enter(BOUNDARY_STOP);
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
  for(int i=0;i<3;i++){digitalWrite(LED_PINS[i],LOW);pinMode(LED_PINS[i],OUTPUT);}
  for(int i=0;i<2;i++)pinMode(ENCODER_PINS[i],INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[0]),leftEncoderPulse,FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[1]),rightEncoderPulse,FALLING);
  encoderSampleAt=millis();
  updateStateLed();
  bootResetType=(RSTC->RSTC_SR & RSTC_SR_RSTTYP_Msk)>>RSTC_SR_RSTTYP_Pos;
  if(bootResetType==2){controlReason="WATCHDOG_RESET";enter(FAULT);}
  else { beginImu(); enter(STOPPED); }
  Serial.print("BOOT reset_type=");Serial.print(bootResetType);
  Serial.println(" WDT_ms=750 (2=watchdog; send f to recover)");
  updateStateLed();
  watchdogReset();

}
void controlRobot() {
  unsigned long now=millis();
  if (Serial.available()) {
    char c=Serial.read();
    if(c=='w' && (state==STOPPED || state==FAULT || state==COMPLETE)){
      enter(STOPPED);updateStateLed();
      Serial.println("WDT_TEST: motors OFF; deliberate hang now; expect reset_type=2, FAULT");
      Serial.flush();watchdogReset();
      while(true){__asm__ volatile("nop");}
    }
    if (c=='s') { enter(STOPPED); return; }
    if (c=='f' && (state==STOPPED || state==COMPLETE || state==FAULT)) {
      runStarted=false; controlReason="OK"; enter(STARTUP); beginImu();
    }
  }
  if (state==STOPPED || state==COMPLETE || state==FAULT) return;
  if (runStarted && now-runSince>=RUN_LIMIT_MS) { enter(COMPLETE); return; }
  bool healthy=rangeValid && now-lastRangeAt<=300;
  if(healthy){
    if(clearanceMm<=APPROACH_MM)approachSlow=true;
    else if(clearanceMm>CRUISE_RESUME_MM)approachSlow=false;
  }
  bool known=lineState[0]>=0 && lineState[1]>=0;
  bool tape=lineState[0]==1 || lineState[1]==1;
  bool obstacle=irBlocked[0] || irBlocked[1] || (healthy && clearanceMm<=AVOID_DISTANCE_MM);
  if (state==STARTUP) {
    if ((imuCalibrated || timedFallback) && now-stateSince>=START_DELAY_MS && known) {
      runSince=now; runStarted=true; enter(READY);
    }
    return;
  }
  // Recover before waiting for sonar. Do not pause an active manoeuvre.
  bool approaching=state==READY || state==DRIVE || state==SENSOR_WAIT;
  if(tape && (approaching || (!boundaryTurn &&
      (state==TURN_STOP || state==TURN || state==TURN_CHECK)))){
    beginBoundary();return;
  }
  if(obstacle && approaching){beginObstacle();return;}
  bool recoveryMotion=state==BOUNDARY_STOP || state==BACKUP ||
                      state==TURN_STOP || state==TURN;
  if(!healthy && !recoveryMotion && state!=SENSOR_WAIT){
    resumeState=state;pausedElapsed=now-stateSince;
    healthyTracking=false;enter(SENSOR_WAIT);
  }
  if(state==SENSOR_WAIT){
    if(!healthy)healthyTracking=false;
    else{
      if(!healthyTracking){healthySince=now;healthyTracking=true;}
      if(now-healthySince>=500){
        state=resumeState;stateSince=now-pausedElapsed;clearTracking=false;
        if(state==DRIVE)resetDriveFeedback();
      }
    }
    return;
  }

  unsigned long elapsed=now-stateSince;
  switch (state) {
    case READY:
      if (obstacle) beginObstacle();
      else if (stableClear(!tape,now)) enter(DRIVE);
      break;
    case DRIVE:
      if (obstacle) beginObstacle();
      else driveWithSpeedFeedback();
      break;
    case BOUNDARY_STOP:
      if (elapsed>=PAUSE_MS) enter(BACKUP);
      break;
    case BACKUP:

      if (elapsed>=REVERSE_MS) enter(TURN_STOP);
      else driveSides(false,false,RECOVERY_PWM);
      break;
    case TURN_STOP:
      if (elapsed>=PAUSE_MS) enter(TURN);
      break;
    case TURN:
      if(elapsed>=MAX_TURN_MS){controlReason="TURN_TIMEOUT";enter(FAULT);break;}

      if (turnDegrees()>=90.0f) enter(TURN_CHECK);
      else driveSides(turnRight,!turnRight,!timedFallback && turnDegrees()>=70.0f ? 180 : TURN_PWM);
      break;
    case TURN_CHECK:

      if (elapsed<500) break;
      if (!tape && !obstacle && clearanceMm>TURN_CLEAR_DISTANCE_MM) {
        if (stableClear(true,now)) enter(READY);
      } else {
        clearTracking=false;
        if (turnSegments>=3) { enter(FAULT); controlReason="PATH_BLOCKED"; }
        else enter(TURN_STOP);
      }
      break;
    default: break;
  }
}
void printStatus() {
  unsigned long now=millis();
  char line[256];
  int cm=(rangeValid && now-lastRangeAt<=300) ? clearanceMm/10 : -1;
  int n=snprintf(line,sizeof(line),"L:%d/%d R:%d/%d IR:%d,%d cm:%d %s %s %s HEAD:%s deg:%d IMU:%s Vcm/s:%d,%d MPWM:%d,%d CTRL:%s RESET:%lu LOOPmax_us:%lu\n",
    lineAvg[0],lineState[0],lineAvg[1],lineState[1],irBlocked[0],irBlocked[1],cm,
    approachSlow?"APPROACH":"CRUISE",NAMES[state],rangeReason,timedFallback ? "TIMED" : "GYRO",(int)roundf(turnDegrees()),imuReason,
    (int)roundf(wheelSpeed[0]),(int)roundf(wheelSpeed[1]),
    motorCommand[0],motorCommand[2],controlReason,(unsigned long)bootResetType,maxLoopUs);

  static char pending[256];
  static int sent=0, length=0;
  if (now-lastPrint>=250 && sent>=length && n>0 && n<(int)sizeof(line)) {
    memcpy(pending,line,n); length=n; sent=0; lastPrint=now;
  }
  int room=Serial.availableForWrite();
  int count=min(room,length-sent);
  if (count>0) { Serial.write((const uint8_t*)pending+sent,count); sent+=count; }

}

void updateStateLed(){
  bool r=false,g=false,b=false;
  bool blink=(millis()%600)<300;
  switch(state){
    case STARTUP: b=blink; break;
    case READY: g=true;b=true;break;
    case DRIVE: g=true;r=approachSlow;break;
    case BOUNDARY_STOP:r=blink;g=blink;break;
    case BACKUP:r=true;b=true;break;
    case TURN_STOP:r=blink;b=blink;break;
    case TURN:b=true;break;
    case TURN_CHECK:r=blink;g=blink;b=blink;break;
    case SENSOR_WAIT:g=blink;b=blink;break;
    case STOPPED:r=true;break;
    case COMPLETE:r=true;g=true;b=true;break;
    case FAULT:r=blink;break;
  }
  digitalWrite(LED_PINS[0],r?HIGH:LOW);
  digitalWrite(LED_PINS[1],g?HIGH:LOW);
  digitalWrite(LED_PINS[2],b?HIGH:LOW);
}
void loop() {
  unsigned long loopStartUs=micros();
  updateTimedHeading();
  if (!triggerHigh) updateImu();
  updateRange();
  updateInputs();
  updateEncoders();
  controlRobot();
  if (!triggerHigh) { printStatus(); updateStateLed(); }
  unsigned long loopUs=micros()-loopStartUs;
  if(loopUs>maxLoopUs)maxLoopUs=loopUs;
  watchdogReset();
}
