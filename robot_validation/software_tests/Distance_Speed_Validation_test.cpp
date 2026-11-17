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

#include <math.h>
const int DIR[4]={7,9,11,13},MOTOR_PWM_PINS[4]={6,8,10,12},FWD[4]={LOW,LOW,HIGH,HIGH};
const float CM_PER_PULSE=3.14159265f*6.5f/10.0f;
const int MIN_PWM=40,MAX_PWM=100;
volatile unsigned long pulses[2]={0,0};
unsigned long previous[2]={0,0},runAt=0,lastSample=0,stopAt=0,activeMs=0;
unsigned long durationMs=10000;
float speed[2]={0,0};int duty[2]={0,0};
unsigned int missing[2]={0,0};
bool running=false,coasting=false,powered=false,regulated=false,distanceRun=false;
const char *mode="IDLE",*result="IDLE";
unsigned int trial=0;
void leftPulse();
void counts(unsigned long &l,unsigned long &r);
void motorOff();
void drive();
void zero();
void row(const char *kind);
void finish(const char *reason);
void start(char command);
int correction(int pwm,float target,float measured);
void setup();
void loop();
void leftPulse(){pulses[0]++;}void rightPulse(){pulses[1]++;}
void counts(unsigned long &l,unsigned long &r){noInterrupts();l=pulses[0];r=pulses[1];interrupts();}
void motorOff(){for(int p:MOTOR_PWM_PINS)analogWrite(p,0);digitalWrite(30,HIGH);digitalWrite(31,LOW);digitalWrite(32,LOW);}
void drive(){for(int i=0;i<4;i++){digitalWrite(DIR[i],FWD[i]);analogWrite(MOTOR_PWM_PINS[i],duty[i<2?0:1]);}}
void zero(){noInterrupts();pulses[0]=pulses[1]=0;interrupts();previous[0]=previous[1]=0;speed[0]=speed[1]=0;}
void row(const char *kind){
 unsigned long l,r;counts(l,r);
 Serial.print(kind);Serial.print(',');Serial.print(trial);Serial.print(',');Serial.print(mode);Serial.print(',');
 Serial.print(millis()-runAt);Serial.print(',');Serial.print(running?millis()-runAt:activeMs);Serial.print(',');
 Serial.print(l);Serial.print(',');Serial.print(r);Serial.print(',');Serial.print(l*CM_PER_PULSE,2);Serial.print(',');Serial.print(r*CM_PER_PULSE,2);Serial.print(',');
 Serial.print(speed[0],2);Serial.print(',');Serial.print(speed[1],2);Serial.print(',');Serial.print(powered&&running?duty[0]:0);Serial.print(',');Serial.print(powered&&running?duty[1]:0);Serial.print(',');Serial.println(result);
}
void finish(const char *reason){
 if(!running)return;
 motorOff();activeMs=millis()-runAt;stopAt=millis();running=false;coasting=true;result=reason;row("STOP");
}
void start(char command){
 if(running||coasting){Serial.println("# Wait until END before starting a trial");return;}
 zero();trial++;result="RUNNING";missing[0]=missing[1]=0;
 powered=command!='m';regulated=command=='c'||command=='f'||command=='d';distanceRun=command=='d';
 int pwm=0;durationMs=10000;
 if(command=='m'){mode="ROLL";durationMs=120000;}
 if(command=='c'){mode="SLOW";pwm=65;}
 if(command=='f'){mode="CRUISE";pwm=85;}
 if(command=='d'){mode="DISTANCE";pwm=85;durationMs=15000;}
 if(command>='1'&&command<='3'){
   pwm=55+20*(command-'1');durationMs=5000;
   mode=command=='1'?"PWM55":command=='2'?"PWM75":"PWM95";
 }
 duty[0]=duty[1]=pwm;runAt=lastSample=millis();activeMs=0;running=true;
 digitalWrite(30,command=='c'?HIGH:LOW);digitalWrite(31,HIGH);digitalWrite(32,LOW);
 row("START");if(powered)drive();
}
int correction(int pwm,float target,float measured){
 float error=target-measured;if(fabsf(error)<=2.0f)return pwm;
 int step=constrain((int)roundf(error*0.5f),-12,8);return constrain(pwm+step,MIN_PWM,MAX_PWM);
}
void setup(){
 for(int i=0;i<4;i++){digitalWrite(MOTOR_PWM_PINS[i],LOW);pinMode(MOTOR_PWM_PINS[i],OUTPUT);digitalWrite(DIR[i],FWD[i]);pinMode(DIR[i],OUTPUT);}
 analogWriteResolution(8);for(int p=30;p<=32;p++)pinMode(p,OUTPUT);motorOff();
 pinMode(2,INPUT_PULLUP);pinMode(3,INPUT_PULLUP);
 attachInterrupt(digitalPinToInterrupt(2),leftPulse,FALLING);attachInterrupt(digitalPinToInterrupt(3),rightPulse,FALLING);
 Serial.begin(115200);
 Serial.println("kind,trial,mode,elapsed_ms,active_ms,left_pulses,right_pulses,left_cm,right_cm,left_cm_s,right_cm_s,left_pwm,right_pwm,result");
 Serial.println("# m=manual roll; c=slow10s; f=cruise10s; d=100cm; 1/2/3=PWM55/75/95; s=stop; z=zero");
}
void loop(){
 while(Serial.available()){
  char c=Serial.read();if(c=='s')finish("MANUAL_STOP");
  else if(c=='z'&&!running&&!coasting)zero();
  else if(c=='m'||c=='c'||c=='f'||c=='d'||c=='1'||c=='2'||c=='3')start(c);
 }
 unsigned long now=millis();
 if(running){
  unsigned long l,r;counts(l,r);
  if(distanceRun && (l+r)*CM_PER_PULSE/2.0f>=100)finish("DISTANCE_REACHED");
  else if(now-runAt>=durationMs)finish(distanceRun?"TIMEOUT":"TIME_COMPLETE");
 }
 if(running && now-lastSample>=1000){
  unsigned long count[2];counts(count[0],count[1]);unsigned long dt=now-lastSample;
  for(int i=0;i<2;i++){
   unsigned long delta=count[i]-previous[i];previous[i]=count[i];speed[i]=delta*CM_PER_PULSE*1000.0f/dt;
   missing[i]=delta==0?missing[i]+1:0;
   if(powered&&missing[i]>=2){finish("NO_PULSES");break;}
   if(regulated&&delta>0)duty[i]=correction(duty[i],strcmp(mode,"SLOW")==0?10.0f:20.0f,speed[i]);
  }
  lastSample=now;if(running){if(powered)drive();row("SAMPLE");}
 }
 if(coasting && now-stopAt>=1000){coasting=false;row("END");mode="IDLE";}
}

int main(){
 setup();for(int p:MOTOR_PWM_PINS)assert(pwmValues[p]==0);
 start('f');assert(running && duty[0]==85);
 for(int t=1;t<=9;t++){fakeNow=t*1000;pulses[0]=pulses[1]=10*t;loop();assert(running);}
 fakeNow=10000;pulses[0]=pulses[1]=98;loop();assert(!running&&coasting&&activeMs==10000);
 for(int p:MOTOR_PWM_PINS)assert(pwmValues[p]==0);
 fakeNow=11000;loop();assert(!coasting);
 start('d');pulses[0]=pulses[1]=49;fakeNow=11010;loop();assert(!running&&strcmp(result,"DISTANCE_REACHED")==0);
 fakeNow=12010;loop();start('m');for(int p:MOTOR_PWM_PINS)assert(pwmValues[p]==0);
 pulses[0]=pulses[1]=49;fakeNow=16000;Serial.input="s";loop();assert(!running&&coasting);
 fakeNow=17000;loop();start('c');fakeNow=18000;loop();assert(running);
 fakeNow=19000;loop();assert(!running&&strcmp(result,"NO_PULSES")==0);
 puts("PASS: no auto-start, timed run, encoder distance stop, coast capture, manual rollout and missing-pulse stop");
}
