// Review refactor prepared 2026-11-10 with AI assistance.
// Derived from the existing shared Autonomous_IMU_90 sketch; package owner is reviewer, not an authorship claim.
// Compile verification is separate from hardware validation.
#pragma once
#include <Arduino.h>
// Conversion helper only. Interrupt acquisition remains in tests/Encoder_Test.
// Pulse calibration is provisional. Not called by autonomous navigation.
struct WheelTravel { float leftCm; float rightCm; };
WheelTravel wheelTravelFromCounts(unsigned long left, unsigned long right);
bool wheelEncoderFeedbackIntegrated();
