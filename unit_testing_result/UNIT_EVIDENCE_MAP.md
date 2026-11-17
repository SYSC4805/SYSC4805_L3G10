# Unit checks

| Unit | Handler/component | Evidence |
| --- | --- | --- |
| U1 | driveSides, motorsOff | Movement, stop assertions |
| U2 | updateInputs | Surface readings, boundary test |
| U3 | HeadingProvider | Modelled turn/pause tests |
| U4 | Pulse handlers, updateEncoders | Manual counts, test harness |
| U5 | updateInputs | IR recovery observations |
| U6 | controlRobot | Turn/manual-stop tests |
| U7 | adjustedPwm, driveWithSpeedFeedback | Deadband/PWM tests |
| U8 | echoChanged, updateRange | Slowdown, stale-range stop |
| U9 | updateStateLed | 13 patterns, both phases |
| F1 | controlRobot, watchdogSetup, loop | Reset logs, watchdog/timer tests |
| H1/P1 | Power/plow | Assembly reported |

Observations: PROGRESS_LOG.md. Software results: CURRENT_SOURCE_TEST_RESULTS.txt. Open acceptance checks: REQUIREMENT_EVIDENCE_MATRIX.md.
