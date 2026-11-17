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
