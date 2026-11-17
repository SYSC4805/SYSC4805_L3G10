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

 runStarted=false;Serial.input="";fakeNow=18000;healthy();
 enter(DRIVE);tick();for(int p:EN_PINS)assert(pwmValues[p]>0);
 lastRangeAt=fakeNow-301;tick();assert(state==SENSOR_WAIT);
 for(int p:EN_PINS)assert(pwmValues[p]==0);
 puts("PASS: stale range over 300ms stops all four PWM outputs");
 fakeNow=19000;healthy();enter(DRIVE);lineState[0]=1;tick();
 assert(state==BOUNDARY_STOP);for(int p:EN_PINS)assert(pwmValues[p]==0);
 fakeNow=19299;healthy();tick();assert(state==BOUNDARY_STOP);
 fakeNow=19300;healthy();tick();assert(state==BACKUP);
 puts("PASS: boundary stop and implemented 300ms reverse delay. R4 50ms criterion remains unmet");
 runStarted=true;runSince=0;fakeNow=RUN_LIMIT_MS;healthy();enter(DRIVE);tick();
 assert(state==COMPLETE);for(int p:EN_PINS)assert(pwmValues[p]==0);
 puts("PASS: 290000ms software operating timer stops all outputs");
 assert(adjustedPwm(85,20,20)==85);
 assert(adjustedPwm(99,20,0)==100);
 assert(adjustedPwm(41,10,100)==40);
 assert(adjustedPwm(85,20,100)<85);
 puts("PASS: feedback deadband, upper/lower PWM clamps and high-estimate duty reduction");
 struct LedCase{State s;bool slow;int r,g,b;bool flashing;};
 LedCase cases[]={{STARTUP,false,0,0,1,true},{READY,false,0,1,1,false},
 {DRIVE,false,0,1,0,false},{DRIVE,true,1,1,0,false},{BOUNDARY_STOP,false,1,1,0,true},
 {BACKUP,false,1,0,1,false},{TURN_STOP,false,1,0,1,true},{TURN,false,0,0,1,false},
 {TURN_CHECK,false,1,1,1,true},{SENSOR_WAIT,false,0,1,1,true},{STOPPED,false,1,0,0,false},
 {COMPLETE,false,1,1,1,false},{FAULT,false,1,0,0,true}};
 for(auto c:cases){state=c.s;slowMode=c.slow;
  for(int phase=0;phase<2;phase++){fakeNow=phase?300:0;updateStateLed();bool on=!c.flashing||phase==0;
   assert(pinValues[30]==(on?c.r:0));assert(pinValues[31]==(on?c.g:0));assert(pinValues[32]==(on?c.b:0));}}
 puts("PASS: all 13 state/mode LED patterns at both blink phases. Physical latency not measured");

}
