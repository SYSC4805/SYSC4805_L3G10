// AJ Donald 101259149

// Test using two line sensors
const int PINS[2] = {A1, A0}; // LEFT, RIGHT
const int BLACK[2] = {3400, 3400};
const int CLEAR[2] = {3100, 3100};
const int ENABLES[4] = {6, 8, 10, 12};
int samples[2][4] = {};
int sums[2] = {}, averages[2] = {}, raw[2] = {};
int classification[2] = {-1, -1}; // -1 unknown, 0 floor, 1 tape
int candidate[2] = {-1, -1}, confirmations[2] = {};
int sampleIndex = 0, filled = 0;
unsigned long sampledAt = 0, printedAt = 0;

void setup() {
  for (int i=0; i<4; i++) {
    digitalWrite(ENABLES[i], LOW);
    pinMode(ENABLES[i], OUTPUT);
  }
  analogReadResolution(12);
  Serial.begin(115200);
  Serial.println("LEFT A1 | RIGHT A0. Labels provisional; compare floor and tape readings.");
}
void loop() {
  unsigned long now = millis();
  if (now-sampledAt >= 5) {
    sampledAt = now;
    if (filled < 4) filled++;
    for (int i=0; i<2; i++) {
      analogRead(PINS[i]); // Discard first conversion after switching channels.
      raw[i] = analogRead(PINS[i]);
      sums[i] -= samples[i][sampleIndex];
      samples[i][sampleIndex] = raw[i];
      sums[i] += raw[i];
      averages[i] = sums[i]/filled;
      if (filled < 4) continue;
      int next = averages[i]>BLACK[i] ? 1 : averages[i]<CLEAR[i] ? 0 : -1;
      if (next < 0) { confirmations[i]=0; candidate[i]=-1; }
      else {
        if (next!=candidate[i]) { candidate[i]=next; confirmations[i]=0; }
        if (confirmations[i]<3) confirmations[i]++;
        if (confirmations[i]>=3) classification[i]=next;
      }
    }
    sampleIndex=(sampleIndex+1)%4;
  }
  if (filled==4 && now-printedAt>=250) {
    printedAt=now;
    for (int i=0; i<2; i++) {
      Serial.print(i==0 ? "LEFT A1: " : " | RIGHT A0: ");
      Serial.print(averages[i]);
      Serial.print(classification[i]<0 ? " UNKNOWN" : classification[i] ? " TAPE" : " FLOOR");
    }
    Serial.println();
  }
}
