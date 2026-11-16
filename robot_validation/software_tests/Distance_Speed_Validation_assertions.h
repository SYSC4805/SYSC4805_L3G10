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
