// AJ Donald 101259149 

// Boundary sensor implementation
#include "BoundarySensors.h"

void updateBoundarySensors() {
  unsigned long now=millis();
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
