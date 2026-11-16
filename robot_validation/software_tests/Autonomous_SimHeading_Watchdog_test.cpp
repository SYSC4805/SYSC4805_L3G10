#include <cmath>
#include <cassert>
#include <cstring>
#include <cstdio>
#include <cstdint>
#include <algorithm>
#include <string>
using std::min;using std::max;
#define constrain(x,a,b) (min(max((x),(a)),(b)))
const int HIGH=1,LOW=0,INPUT=0,OUTPUT=1,INPUT_PULLUP=2,FALLING=3,CHANGE=4,A0=54,A1=55;
uint32_t fakeNow=0;unsigned int feeds=0;uint32_t wdtConfigured=0;
unsigned long millis(){return fakeNow;} unsigned long micros(){return fakeNow*1000;}
void noInterrupts(){} void interrupts(){} void pinMode(int,int){}
void analogWriteResolution(int){} void analogReadResolution(int){}
int pinValues[80]={},pwmValues[80]={};
void digitalWrite(int p,int v){pinValues[p]=v;} int digitalRead(int p){return pinValues[p];}
int analogRead(int){return 2000;} void analogWrite(int p,int v){pwmValues[p]=v;}
int digitalPinToInterrupt(int p){return p;}void attachInterrupt(int,void (*)(),int){}
void watchdogEnable(uint32_t ms){wdtConfigured=ms;}void watchdogReset(){feeds++;}
struct ResetRegisters{uint32_t RSTC_SR=0;} resetRegisters;
ResetRegisters *RSTC=&resetRegisters;
const uint32_t RSTC_SR_RSTTYP_Msk=7<<8,RSTC_SR_RSTTYP_Pos=8;
struct MockSerial{
 std::string input;void begin(int){}int available(){return input.size();}
 char read(){char c=input[0];input.erase(0,1);return c;}
 int availableForWrite(){return 64;}void write(const uint8_t*,int){}
 template<class T>void print(T){}template<class T>void print(T,int){}
 template<class T>void println(T){}void println(){}void flush(){}
}Serial;
#include "HeadingProvider.h"
#include <math.h>

const int DIR_PINS[] = {7,9,11,13};
const int EN_PINS[] = {6,8,10,12};
const int FORWARD_LEVEL[] = {LOW,LOW,HIGH,HIGH};
const bool IS_LEFT[] = {true,true,false,false};
const int CRUISE_PWM=85, SLOW_PWM=65, RECOVERY_PWM=100;
const int TURN_PWM=200;
const unsigned long LEFT_90_MS=950, RIGHT_90_MS=950;
HeadingProvider heading(LEFT_90_MS,RIGHT_90_MS);
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
const int FRONT_OFFSET_MM=0;
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
void enter(State next);
const char *NAMES[]={"STARTUP","READY","DRIVE","RECOVERY_STOP","BACKUP",
                    "TURN_STOP","TURN","SENSOR_WAIT","STOPPED","COMPLETE","FAULT","TURN_CHECK"};
State state=STARTUP, resumeState=READY;
unsigned long stateSince=0, runSince=0, pausedElapsed=0, healthySince=0;
unsigned long clearSince=0, lastPrint=0;
bool runStarted=false, healthyTracking=false, clearTracking=false;
bool turnRight=true, boundaryTurn=false;

const int ENCODER_PINS[2]={2,3};
const int LED_PINS[3]={30,31,32};
const float CM_PER_PULSE=3.14159265f*6.5f/10.0f;
const float CRUISE_CM_S=20.0f, SLOW_CM_S=10.0f;
const float SPEED_DEADBAND=2.0f;

const bool ENABLE_OVERSPEED_STOP=false;
const int DRIVE_PWM_MIN=40, DRIVE_PWM_MAX=100;
const unsigned long SPEED_WINDOW_MS=1000;
volatile unsigned long encoderCount[2]={0,0};
unsigned long encoderPrevious[2]={0,0}, encoderSampleAt=0;
unsigned long drivePrevious[2]={0,0}, driveSampleAt=0;
float wheelSpeed[2]={0,0};
int drivePwm[2]={CRUISE_PWM,CRUISE_PWM};
bool driveWasSlow=false;
unsigned int missingWindows[2]={0,0}, overspeedWindows[2]={0,0};
const char *controlReason="OK";
void watchdogSetup();
void leftEncoderPulse();
void rightEncoderPulse();
void encoderSnapshot(unsigned long &left,unsigned long &right);
void updateEncoders();
void resetDriveFeedback();
int adjustedPwm(int current,float target,float measured);
void driveWithSpeedFeedback();
void echoChanged();
void updateRange();
void motorsOff();
void driveSides(bool leftForward, bool rightForward, int pwm);
void enter(State next);
void updateInputs();
bool stableClear(bool clear, unsigned long now);
void beginBoundary();
void beginObstacle();
void setup();
void controlRobot();
void printStatus();
void updateStateLed();
void loop();
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
  driveSampleAt=millis();driveWasSlow=slowMode;
  for(int i=0;i<2;i++){
    drivePwm[i]=slowMode?SLOW_PWM:CRUISE_PWM;
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
  unsigned long now=millis();
  if(slowMode!=driveWasSlow){

    resetDriveFeedback();
  }
  unsigned long dt=now-driveSampleAt;
  if(dt>=SPEED_WINDOW_MS){
    unsigned long count[2];encoderSnapshot(count[0],count[1]);
    for(int i=0;i<2;i++){
      unsigned long delta=count[i]-drivePrevious[i];
      float measured=delta*CM_PER_PULSE*1000.0f/dt;
      drivePrevious[i]=count[i];
      missingWindows[i]=delta==0?missingWindows[i]+1:0;
      overspeedWindows[i]=(ENABLE_OVERSPEED_STOP && measured>30.0f)
        ? min(overspeedWindows[i]+1,2U) : 0;

      if(missingWindows[i]>=2 || (ENABLE_OVERSPEED_STOP && overspeedWindows[i]>=2)){
        controlReason=missingWindows[i]>=2?"ENCODER_NO_PULSES":"ENCODER_OVERSPEED";
        enter(FAULT);return;
      }
      if(delta>0)drivePwm[i]=adjustedPwm(drivePwm[i],slowMode?SLOW_CM_S:CRUISE_CM_S,measured);
    }
    driveSampleAt=now;
  }
  for(int i=0;i<4;i++){
    digitalWrite(DIR_PINS[i],FORWARD_LEVEL[i]);
    analogWrite(EN_PINS[i],drivePwm[IS_LEFT[i]?0:1]);
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
  heading.setMotion(false,false,millis());
  for (int i=0;i<4;i++) analogWrite(EN_PINS[i],0);
}
void driveSides(bool leftForward, bool rightForward, int pwm) {
  heading.setMotion(leftForward!=rightForward && pwm>0,leftForward,millis());
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
  if (next==TURN) {turnSegments++;heading.beginTurn(turnRight,millis());}
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
  turnRight=irBlocked[0] || !irBlocked[1];
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
  else stateSince=millis();
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
      runStarted=false; controlReason="OK"; enter(STARTUP);
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
        if(state==DRIVE)resetDriveFeedback();
      }
    }
    return;
  }

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

      if (heading.turnDegrees()>=90.0f) enter(TURN_CHECK);
      else driveSides(turnRight,!turnRight,TURN_PWM);
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
  int n=snprintf(line,sizeof(line),"L:%d/%d R:%d/%d IR:%d,%d cm:%d %s %s %s HEAD:SIM_TIME SIMdeg:%d Vcm/s:%d,%d FPWM:%d,%d CTRL:%s RESET:%lu LOOPmax_us:%lu\n",
    lineAvg[0],lineState[0],lineAvg[1],lineState[1],irBlocked[0],irBlocked[1],cm,
    slowMode ? "SLOW" : "FAST",NAMES[state],rangeReason,(int)roundf(heading.turnDegrees()),
    (int)roundf(wheelSpeed[0]),(int)roundf(wheelSpeed[1]),
    state==DRIVE?drivePwm[0]:0,state==DRIVE?drivePwm[1]:0,controlReason,(unsigned long)bootResetType,maxLoopUs);

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
    case DRIVE: g=true;r=slowMode;break;
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
  heading.update(millis());
  updateRange();
  updateInputs();
  updateEncoders();
  controlRobot();
  if (!triggerHigh) { printStatus(); updateStateLed(); }
  unsigned long loopUs=micros()-loopStartUs;
  if(loopUs>maxLoopUs)maxLoopUs=loopUs;
  watchdogReset();
}

void tick(){heading.update(millis());controlRobot();}
void healthy(){rangeValid=true;lastRangeAt=fakeNow;clearanceMm=600;lineState[0]=lineState[1]=0;irBlocked[0]=irBlocked[1]=false;}
int main(){
 watchdogSetup();assert(wdtConfigured==750);
 resetRegisters.RSTC_SR=2<<8;setup();assert(state==FAULT && strcmp(controlReason,"WATCHDOG_RESET")==0);
 fakeNow=10000;healthy();tick();assert(state==FAULT);for(int p:EN_PINS)assert(pwmValues[p]==0);
 Serial.input="f";tick();assert(state==STARTUP);
 fakeNow=13000;healthy();tick();assert(state==READY);
 boundaryTurn=true;turnRight=true;enter(TURN);tick();
 fakeNow=13000+RIGHT_90_MS/2;healthy();tick();assert(state==TURN && fabsf(heading.turnDegrees()-45)<0.1f);
 rangeValid=false;tick();assert(state==SENSOR_WAIT);
 fakeNow=14000;healthy();tick();fakeNow=14500;healthy();tick();assert(state==TURN);
 tick();fakeNow=14500+RIGHT_90_MS/2;healthy();tick();assert(state==TURN_CHECK);
 assert(fabsf(heading.turnDegrees()-90)<0.1f);
 unsigned int before=feeds;loop();assert(feeds==before+1);
 Serial.input="s";tick();assert(state==STOPPED);
 puts("PASS: watchdog hook/boot hold/restart/feed policy; modelled heading pauses through sensor wait and ends turn");
}
