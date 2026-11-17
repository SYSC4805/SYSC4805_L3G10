# Progress through November 16, 2026

Summary of team reports and chat excerpts, not a raw trial log. Individual hours and contributions were not recorded.

| Item | Evidence | Still unverified |
| --- | --- | --- |
| Motors/navigation | Movement and recovery operated | Containment and no-contact trials |
| Lines | Left floor 3086–3105, tape 3951–4011. Right floor 2804–2840, tape 3906–3957 | Timing and other surfaces |
| Encoders | Ten-revolution counts: left 100/100/100, right 100/101/103 | Physical speed/distance |
| Powered encoders | PWM75, 3s: 512/154 pulses, 513/155 after coast | Matching ground measurement |
| IR/range | Detection, slowdown and recovery reported | Accuracy, latency, cube rejection |
| LED | Colours/states verified by team | Update latency |
| Turning | Approximate quarter turns at PWM200 | Measured ±5° accuracy |
| IMU | I2C unreliable, SIM_TIME used on reported TA advice | Physical heading |
| Plow | Original built, smaller version planned | Dimensions and clearing |

Current turns use 950ms per side. Encoders use provisional 10 pulses/rev and 6.5cm wheels. Overspeed cutoff is disabled, with 20/10cm/s targets and missing-pulse protection retained.

Two post-reset captures show RESET:2, FAULT and FPWM:0,0. The timestamped excerpt spans 11:42:39.466–11:42:42.953 after boot. It does not establish reset latency or physical outputs during reset.

Host checks passed. Hardware acceptance remains open. November 16 planned value: CAD 7,764.29 (62.1%). Actual hours and full earned value are unavailable.

Testing resumes after the November 19 submission. Demo/code/report: December 10. Presentation: December 11.
