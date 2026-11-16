# Unit evidence

Hardware observations are summarized in PROGRESS_LOG.md. Software results are in CURRENT_SOURCE_TEST_RESULTS.txt. Full acceptance remains open as listed in REQUIREMENT_EVIDENCE_MATRIX.md.

| Unit | Handler or component | Available evidence |
| --- | --- | --- |
| U1 motors | driveSides, motorsOff | Movement observations, stop assertions |
| U2 lines | updateInputs | Surface readings, boundary control test |
| U3 heading | HeadingProvider | Modelled turn and pause/resume tests |
| U4 encoders | Pulse handlers, updateEncoders | Manual counts, distance test harness |
| U5 IR | updateInputs | Stop/recovery observations |
| U6 motion | controlRobot | Turn completion and manual-stop tests |
| U7 speed | adjustedPwm, driveWithSpeedFeedback | Deadband and PWM-limit tests |
| U8 range | echoChanged, updateRange | Slowdown observations, stale-range stop |
| U9 LED | updateStateLed | 13 patterns at both blink phases |
| F1 framework | controlRobot, watchdogSetup, loop | Reset excerpt, watchdog and timer tests |
| H1 power | Physical assembly | Team assembly report |
| P1 plow | Physical assembly | Team reports built plow |
