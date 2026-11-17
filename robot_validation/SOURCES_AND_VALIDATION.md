# Validation

Shared, AI-assisted code. Historical Due builds:

| Sketch | Bytes |
| --- | ---: |
| Autonomous_SimHeading_Watchdog | 45,880 |
| Distance_Speed_Validation | 32,572 |
| Turn_Validation | 30,408 |

Host checks and source hashes: [results](../unit_testing_result/CURRENT_SOURCE_TEST_RESULTS.txt). These do not prove hardware accuracy or timing. The historical build confirmed the watchdogSetup override.

Sources: [Arduino watchdog](https://github.com/arduino/ArduinoCore-sam/blob/master/cores/arduino/watchdog.cpp), [main loop](https://github.com/arduino/ArduinoCore-sam/blob/master/cores/arduino/main.cpp), [SAM3X/A datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-11057-32-bit-Cortex-M3-Microcontroller-SAM3X-SAM3A_Datasheet.pdf).
