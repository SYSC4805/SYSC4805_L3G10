# Run the sketches

Arduino Due, Serial Monitor 115200 baud. Open the matching `.ino` folder. Keep `HeadingProvider.h` beside its sketch.

| Sketch | Commands |
| --- | --- |
| Autonomous | Auto-start. `s` stop, `f` restart, `w` watchdog test while stopped |
| Distance/speed | `m` manual roll, `c/f` slow/cruise for 10s, `d` encoder 100cm, `1/2/3` PWM 55/75/95 for 5s, `s` stop, `z` idle reset |
| Turn | `l/r` left/right, `s` stop. Wait for READY |

Distance/turn tests ignore obstacles and tape. Use a clear area, accessible power switch and wheels-raised initial check.

## Record results

- Distance: five measured 1m rolls.
- Speed: five slow and five cruise trials, matching physical distance, time and encoder counts.
- Turns: five per direction, measured against an external reference.
- Watchdog: disconnect motor power, send `s`, then `w`. Time hang-to-reset. Expect RESET:2 and stopped FAULT until `f`. Repeat five times. Separately verify motor outputs through reset with power connected.

STOP excludes coast. END includes 1s coast. Save logs, video, firmware version and conditions. Enter independent measurements in `measured_results.csv`, then run:

```sh
python3 analyze_results.py measured_results.csv
```

Targets: ≤5cm error per metre, 10/20 ±2cm/s, encoder speed error ≤10%, turns 90 ±5°.

SIM_TIME is modelled. Encoder calibration is provisional. Overspeed cutoff is disabled. Average speed does not prove the 30cm/s peak limit. The 300ms reverse pause misses R4's 50ms target. A stopped watchdog test does not prove moving-fault stopping time.
