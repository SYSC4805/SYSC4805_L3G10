# Sources and validation

Shared, AI-assisted code. Historical Due builds:

| Sketch | Bytes |
| --- | ---: |
| Autonomous_SimHeading_Watchdog | 45,880 |
| Distance_Speed_Validation | 32,572 |
| Turn_Validation | 30,408 |

Current host checks passed for watchdog hold/restart, sensor waits, motor stops, timer, PWM limits, LED patterns, turn control and analysis calculations. Results and source hashes: `../evidence/CURRENT_SOURCE_TEST_RESULTS.txt`.

These are software checks, not physical acceptance results. The strong `watchdogSetup` override was confirmed in the historical build.

References:

- [Arduino watchdog implementation](https://github.com/arduino/ArduinoCore-sam/blob/master/cores/arduino/watchdog.cpp)
- [Arduino main loop](https://github.com/arduino/ArduinoCore-sam/blob/master/cores/arduino/main.cpp)
- [SAM3X/A datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-11057-32-bit-Cortex-M3-Microcontroller-SAM3X-SAM3A_Datasheet.pdf)
