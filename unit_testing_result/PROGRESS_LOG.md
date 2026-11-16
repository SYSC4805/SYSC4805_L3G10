# Progress summary

Condensed from team reports and chat logs through November 16, 2026. This is not a complete raw test log. Earlier test dates, individual hours and contributions are not established here.

| Item | Recorded result | Limit |
| --- | --- | --- |
| Motors/navigation | Forward, reverse, turns and recovery operated | Formal containment and no-contact trials incomplete |
| Lines | Left floor 3086–3105, tape 3951–4011. Right floor 2804–2840, tape 3906–3957 | Surface-specific readings, not response-time proof |
| Encoders | Ten-revolution trials: left 100/100/100, right 100/101/103 | Current 10 pulses/rev and 6.5cm diameter remain provisional |
| Powered encoders | PWM 75, 3s: 512/154 pulses, then 513/155 after coast | No matching physical distance/time measurement |
| IR/range | Team confirmed detection, slowdown and recovery | Range accuracy, latency and cube rejection unmeasured |
| LED | Team verified Due-driven colours and states | Update latency unmeasured |
| Turning | PWM 200 gave approximate quarter turns | Visual observations, not measured ±5° acceptance |
| IMU | Motor-on I2C communication unresolved | SIM_TIME used following reported TA advice |
| Plow | Original plow built, smaller version planned | Dimensions and clearing performance unverified |

## Changes and later evidence

- Turning moved from PWM 255 to 200. Left and right durations were adjusted from 900ms to 950ms. SIM_TIME angles are model outputs.
- Encoder overspeed cutoff was disabled for testing. Speed targets remain 20/10cm/s, with missing-pulse protection.
- A 750ms watchdog was added. Two supplied post-reset captures show FAULT, RESET:2 and FPWM:0,0.
- The timestamped capture spans 11:42:39.466–11:42:42.953 after boot. It does not measure reset latency or prove motor output levels during reset.
- Current-source host checks passed. Physical speed, turn accuracy, watchdog timing and full arena acceptance remain open.
- November 16 planned value is CAD 7,764.29, or 62.1% of the baseline. Actual hours and full earned value are unavailable.

Remaining hardware tests are planned after the November 19 submission. Demo, code and final report are due December 10. Presentation is December 11.
