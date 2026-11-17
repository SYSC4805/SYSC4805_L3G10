// AJ DOnald 101259149

// implementation file for IR sensors
#include "ObstacleSensors.h"

void updateObstacleSensors() {
  unsigned long now=millis();
  for (int i=0;i<2;i++) {
    bool raw=digitalRead(IR_PINS[i])==LOW;
    if (raw!=irCandidate[i]) { irCandidate[i]=raw; irSince[i]=now; }
    if (now-irSince[i]>=(raw ? 20UL : 50UL)) irBlocked[i]=raw;
  }
}
