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

const int DIR[4]={7,9,11,13},MOTOR_PWM_PINS[4]={6,8,10,12},FWD[4]={LOW,LOW,HIGH,HIGH};
const int TURN_PWM=200;
const unsigned long LEFT_90_MS=950,RIGHT_90_MS=950;
HeadingProvider heading(LEFT_90_MS,RIGHT_90_MS);
bool running=false,settling=false,rightTurn=true;
unsigned long startAt=0,stopAt=0,lastLog=0;
unsigned int trial=0;
void off();
void row(const char *event);
void start(bool right);
void setup();
void loop();
void off(){heading.setMotion(false,false,millis());for(int p:MOTOR_PWM_PINS)analogWrite(p,0);digitalWrite(32,LOW);digitalWrite(30,HIGH);}
void row(const char *event){
 Serial.print(event);Serial.print(',');Serial.print(trial);Serial.print(',');Serial.print(rightTurn?"RIGHT":"LEFT");Serial.print(',');
 Serial.print(millis()-startAt);Serial.print(',');Serial.print(heading.turnDegrees(),2);Serial.print(',');Serial.print(TURN_PWM);Serial.println(",SIM_TIME");
}
void start(bool right){
 if(running||settling){Serial.println("# Wait for READY");return;}
 rightTurn=right;trial++;startAt=lastLog=millis();heading.beginTurn(right,startAt);
 digitalWrite(30,LOW);digitalWrite(32,HIGH);row("START");
 for(int i=0;i<4;i++){bool forward=i<2?right:!right;digitalWrite(DIR[i],forward?FWD[i]:!FWD[i]);analogWrite(MOTOR_PWM_PINS[i],TURN_PWM);}
 heading.setMotion(true,right,millis());running=true;
}
void setup(){
 for(int i=0;i<4;i++){digitalWrite(MOTOR_PWM_PINS[i],LOW);pinMode(MOTOR_PWM_PINS[i],OUTPUT);pinMode(DIR[i],OUTPUT);}
 analogWriteResolution(8);for(int p=30;p<=32;p++){digitalWrite(p,LOW);pinMode(p,OUTPUT);}off();
 Serial.begin(115200);Serial.println("event,trial,direction,elapsed_ms,simulated_degrees,pwm,heading_source");
 Serial.println("# l=left r=right s=stop; modelled degrees are NOT measured degrees");
}
void loop(){
 heading.update(millis());
 while(Serial.available()){
  char c=Serial.read();
  if(c=='s'){off();running=false;settling=true;stopAt=millis();row("MANUAL_STOP");}
  else if(c=='l')start(false);else if(c=='r')start(true);
 }
 if(running){
  if(heading.turnDegrees()>=90.0f || millis()-startAt>=3000){
   off();running=false;settling=true;stopAt=millis();row(heading.turnDegrees()>=90?"STOP":"TIMEOUT");
  }else if(millis()-lastLog>=100){lastLog=millis();row("SAMPLE");}
 }
 if(settling && millis()-stopAt>=1000){settling=false;row("READY");}
}

int main(){
 setup();assert(!running);start(true);assert(running);
 fakeNow=RIGHT_90_MS/2;loop();assert(running&&fabsf(heading.turnDegrees()-45)<0.1f);
 fakeNow=RIGHT_90_MS;loop();assert(!running&&settling);for(int p:MOTOR_PWM_PINS)assert(pwmValues[p]==0);
 fakeNow=RIGHT_90_MS+1000;loop();assert(!settling);start(false);fakeNow=2350;loop();
 Serial.input="s";loop();assert(!running);for(int p:MOTOR_PWM_PINS)assert(pwmValues[p]==0);
 puts("PASS: one-turn command, simulated angle, automatic stop/settle, left turn and manual stop");
}
