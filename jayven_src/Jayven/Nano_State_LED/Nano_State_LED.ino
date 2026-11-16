// Nano Every: R->D3, G->D5, B->D6, EACH through its own220-ohm resistor.
// DueTX1 D18 -> level shifterLV1/HV1 -> NanoRX D0. Common ground.
// USB Serial115200 logs state changes. Serial1 is hardwareRX/TX pins.
const bool COMMON_ANODE=false; // Set true if shared LED leg connects to5V.
const int RGB[3]={3,5,6};
const byte COLORS[8][3]={
 {100,100,100}, {0,150,0}, {150,100,0}, {150,0,0},
 {150,35,0}, {0,0,150}, {0,150,150}, {150,0,150}
};
const char *NAMES[]={"INIT","DRIVE","SLOW","STOP","REVERSE","TURN","COMPLETE","FAULT"};
byte currentState=0, parser=0, received=0;
bool linked=false, testMode=false, linkLostLogged=false;
unsigned long lastPacket=0, testSince=0;
void show(byte r,byte g,byte b){
 byte values[3]={r,g,b};
 for(int i=0;i<3;i++) analogWrite(RGB[i], COMMON_ANODE ? 255-values[i] : values[i]);
}
void setup(){
 for(int i=0;i<3;i++){
   digitalWrite(RGB[i],COMMON_ANODE ? HIGH : LOW); pinMode(RGB[i],OUTPUT);
 }
 Serial.begin(115200); Serial1.begin(9600);
 show(0,0,0);
 Serial.println("Nano state display. t=color test; n=normal state display.");
}
void loop(){
 unsigned long now=millis();
 if(Serial.available()){
   char c=Serial.read();
   if(c=='t'){testMode=true;testSince=now;Serial.println("LED TEST (display only)");}
   if(c=='n'){testMode=false;Serial.println("LIVE STATE DISPLAY");}
 }
 for(int count=0;count<24 && Serial1.available();count++){
   byte value=Serial1.read();
   if(parser==0){if(value==0xA5)parser=1;}
   else if(parser==1){
     if(value<8){received=value;parser=2;}
     else parser=(value==0xA5)?1:0;
   } else {
     if(value==(byte)(received^0xFF)){
       if(!linked || linkLostLogged || currentState!=received){
         Serial.print(now);Serial.print(" ms | ");Serial.println(NAMES[received]);
       }
       currentState=received;linked=true;lastPacket=now;linkLostLogged=false;
     }
     parser=(value==0xA5)?1:0;
   }
 }
 if(testMode){
   byte index=((now-testSince)/1000)%8;
   show(COLORS[index][0],COLORS[index][1],COLORS[index][2]);
 } else if(!linked || now-lastPacket>1000){
   bool on=(now/250)%2;show(on?150:0,0,on?150:0);
   if(!linkLostLogged){Serial.println("NO DUE STATE LINK");linkLostLogged=true;}
 } else show(COLORS[currentState][0],COLORS[currentState][1],COLORS[currentState][2]);
}
