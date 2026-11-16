# Run the sketches

Select Arduino Due and Serial Monitor at 115200 baud. Open each `.ino` from its matching folder. Keep `HeadingProvider.h` beside the autonomous and turn sketches.

## Controls

| Sketch | Commands |
| --- | --- |
| Autonomous_SimHeading_Watchdog | Auto-starts after startup checks. `s` stop, `f` restart, `w` watchdog test while stopped |
| Distance_Speed_Validation | `m` manual roll, `c` slow 10s, `f` cruise 10s, `d` encoder 100cm, `1/2/3` PWM 55/75/95 for 5s, `s` stop, `z` reset while idle |
| Turn_Validation | `l` left, `r` right, `s` stop. Wait for READY before repeating |

Distance and turn tests ignore obstacles and boundaries. Use a clear area and accessible power switch. Start motor checks with wheels raised.

## Measurements

- Distance: five measured 1m rolls. Compare both encoders with the floor reference.
- Speed: five slow and five cruise trials. Record physical distance, time and counts over the same interval.
- Turns: five left and five right trials. Measure final angle against a floor reference.
- Watchdog: first disconnect motor power, send `s`, then `w`. Record hang-to-reset time. Expect reset_type=2 and FAULT with PWM off until `f`. Repeat five times and separately verify outputs through reset with motor power connected.

STOP excludes coast. END includes one second of coast. Match counts, distance and time consistently. Save raw logs, video, firmware version and test conditions.

## Limits

SIM_TIME is modelled heading. Encoders use provisional 10 pulses/revolution and 6.5cm wheels. Overspeed cutoff is disabled for testing. The 300ms reverse pause does not meet R4's 50ms target. The stopped watchdog test does not prove moving-fault stopping time.

Enter independent measurements in `measured_results.csv`, then run:

```sh
python3 analyze_results.py measured_results.csv
```

Targets: distance error ≤5cm per metre, speed 10/20 ±2cm/s, encoder speed error ≤10%, turns 90 ±5°. Average speed does not prove the 30cm/s peak limit.
