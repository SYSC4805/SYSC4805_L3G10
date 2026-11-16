// Serial: l left, r right, s stop. Use a clear area.
// SIM_TIME is modelled, not measured.
#include "HeadingProvider.h"

const int DIR[4]={7,9,11,13},MOTOR_PWM_PINS[4]={6,8,10,12},FWD[4]={LOW,LOW,HIGH,HIGH};
const int TURN_PWM=200;
const unsigned long LEFT_90_MS=950,RIGHT_90_MS=950;
HeadingProvider heading(LEFT_90_MS,RIGHT_90_MS);
bool running=false,settling=false,rightTurn=true;
unsigned long startAt=0,stopAt=0,lastLog=0;
unsigned int trial=0;
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
