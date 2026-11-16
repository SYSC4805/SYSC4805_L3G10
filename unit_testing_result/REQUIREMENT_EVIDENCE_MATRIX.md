# Requirement evidence

Matches report Table 2.1. Software checks do not establish physical acceptance.

| Requirement | Available evidence | Remaining gap |
| --- | --- | --- |
| R1 startup | 3s delay plus sensor checks | Measured power-on to movement |
| R2 recovery | Team observations, pause/resume tests | Repeated recovery trials |
| R3 dimensions | Assembly reported | Measured envelope and plow |
| R4 reverse ≤50ms | Test confirms 300ms pause | Known mismatch to resolve |
| R5 excursion ≤5cm | Line separation, recovery logic | Measured wheel excursion |
| R6 speed | PWM bounds, manual counts | Physical calibration and peak speed |
| R7 turn ±5° | Timed-turn tests | External angle measurements |
| R8 straight-line drift | No qualifying result | Measured 2m trials |
| R9 ranging/slowdown | Team observations, thresholds | Distance and height trials |
| R10 IR stop/no contact | Team stop/recovery observations | Both sensors' latency and contact trials |
| R11 cube rejection | Mounting strategy | Repeated 20mm cube trials |
| R12 LED | Team checks, 13 software patterns | Physical latency and repetition |
| R13 battery monitor | Removed on reported TA advice | Retain confirmation if available |
| R14 watchdog/loop | Reset excerpt, host tests | Measured recovery and full-run timing |
| R15 290s stop | Software timer test | Physical operating-time check |
| R16 clearing | No qualifying run series | Five arena runs with counts/penalties |

Sources: `PROGRESS_LOG.md`, `watchdog_chat_excerpt.txt`, `CURRENT_SOURCE_TEST_RESULTS.txt` and `CALCULATION_RESULTS.txt`.
