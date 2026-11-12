// Review refactor prepared 2026-11-10 with AI assistance.
// Derived from the existing shared Autonomous_IMU_90 sketch; package owner is reviewer, not an authorship claim.
// Compile verification is separate from hardware validation.
#include "WheelEncoders.h"
WheelTravel wheelTravelFromCounts(unsigned long left, unsigned long right) {
  const float circumferenceCm = 3.14159265358979323846f * 6.5f;
  return {left * circumferenceCm / 11.0f, right * circumferenceCm / 12.0f};
}
bool wheelEncoderFeedbackIntegrated() { return false; }
