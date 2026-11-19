# Approach speed and alternating obstacle turns

Due, Serial115200. f calibrates/starts, s stops. Keep still during calibration.

Cruise starts atPWM150, target25cm/s, ceiling170. At35cm plow clearance or less, approach mode starts atPWM110, target15cm/s, ceiling125. Cruise resumes beyond40cm. Green means cruise, yellow means approach.

Ultrasonic offset4cm: sensor distances are39cm to slow,44cm to resume cruise and24cm to trigger recovery (20cm plow clearance). Either IR input also triggers recovery immediately.

Obstacles: stop, reverse400ms atPWM125, turn, check clearance, resume. New obstacle events alternate left/right, starting left. Extra turns within one recovery keep their direction. Line recovery stays left/both tape -> right, right tape -> left, without advancing the obstacle sequence.

IMU errors select timed fallback. Rear space must be clear. Sensor faults, watchdog and missing-pulse protection remain. Speed targets require floor validation.
