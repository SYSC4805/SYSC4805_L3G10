int main(){
 setup();assert(!running);start(true);assert(running);
 fakeNow=RIGHT_90_MS/2;loop();assert(running&&fabsf(heading.turnDegrees()-45)<0.1f);
 fakeNow=RIGHT_90_MS;loop();assert(!running&&settling);for(int p:MOTOR_PWM_PINS)assert(pwmValues[p]==0);
 fakeNow=RIGHT_90_MS+1000;loop();assert(!settling);start(false);fakeNow=2350;loop();
 Serial.input="s";loop();assert(!running);for(int p:MOTOR_PWM_PINS)assert(pwmValues[p]==0);
 puts("PASS: one-turn command, simulated angle, automatic stop/settle, left turn and manual stop");
}
