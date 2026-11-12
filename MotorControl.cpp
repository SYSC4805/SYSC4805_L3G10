// Review refactor prepared 2026-11-10 with AI assistance.
// Derived from the existing shared Autonomous_IMU_90 sketch; package owner is reviewer, not an authorship claim.
// Compile verification is separate from hardware validation.
#include "MotorControl.h"

void motorsOff() {
  for (int i=0;i<4;i++) analogWrite(EN_PINS[i],0);
}

void driveSides(bool leftForward, bool rightForward, int pwm) {
  for (int i=0;i<4;i++) {
    bool forward=IS_LEFT[i] ? leftForward : rightForward;
    int direction=forward ? FORWARD_LEVEL[i] : !FORWARD_LEVEL[i];
    digitalWrite(DIR_PINS[i],direction);
    analogWrite(EN_PINS[i],pwm);
  }
}
