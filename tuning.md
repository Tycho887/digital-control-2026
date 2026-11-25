# Differential-drive trajectory tuning

Experiment 11 (`cd95a84`) is the baseline for the next trial. The robot drives 2 m out, turns 180° clockwise, drives 2 m back, then turns 180° counterclockwise. The final turn brings the same wheels back to their starting orientation so their floor marks can be compared directly. The observed return error was approximately **1–2 cm by eye**. The two onboard estimates were **3.1 cm and 2.5 cm**. [Trajectory profile](src/src/trajectory.h)

## Changes kept on the robot

| Change | Measurement and decision |
| --- | --- |
| Ramp the turn-rate reference (`3a125f3`) | A step into a four-second turn produced 7.47° of peak estimated heading overshoot. Ramping the first and last 0.5 s and adjusting the peak rate to preserve the 180° command reduced overshoot to 1.13–1.44° in the ramped trials. |
| Brake before the pivot (`6c81e64`) | The straight-leg reference ramps down over its last 0.60 s. A slightly higher cruise speed preserves the planned 2 m. Net forward travel inferred from the two wheels during the pivot fell from 8.93 cm to 4.44 and 4.59 cm on the repeat runs. This measures how far the robot rolled while trying to turn in place. |
| Smooth the right-wheel speed estimate (`2ea9f58`) | Reducing the right encoder filter’s β from 0.20 to 0.15 made its speed estimate less sensitive to individual count changes. The two runs used to retain the setting showed 0.31–0.32 V of outbound cruise command variation and met the corner-travel and heading targets. |
| Separate braking, turning, and settling (`51e346f`, `cd95a84`) | A 0.5 s stop before each pivot gives a point at which to read the end of the straight. A 2.5 s heading hold after each pivot separates peak overshoot from settled error. This makes each phase measurable within the 22.2 s course. |

**Controller settings.** At 1 kHz, the heading lead-lag controller corrects the turn-rate reference, the yaw-rate PI corrects the rate error, and two wheel-speed PIs drive the motors. The wheel PI gains are `Kp = 0.05` left and `0.04` right, with `Ki = 0.10` on both and a ±9 V output limit. The yaw-rate PI uses `Kp = 2.5` and `Ki = 12.5`. The encoder velocity filters use (α, β) of (0.50, 0.10) left and (0.80, 0.15) right. In both experiment 10 repeats the heading settled within 0.5° of the turn target before the return leg, and the logs recorded zero wheel, heading, or yaw-controller saturation flags. We kept those gains for experiment 11. [Firmware tuning values](src/src/tuning.h)

## Geometry and estimator checks

- **Wheel geometry.** The MATLAB trial used a 30.0 mm radius and 9.68 gear ratio together. The robot visibly spun and overcorrected. The same capture (`device-monitor-261118-131743.log`) also recorded zero gyro updates, so the cause cannot be isolated from that run. Experiment 10 restored the 31.5 mm radius and 9.6 ratio and required fresh, calibrated gyro feedback for trajectory tracking. The wheelbase was 140 mm in both configurations. [MATLAB model parameters](week06/init_week06.m) · [Firmware geometry and gyro checks](src/src/basebot_6.ino)
- **IMU alignment and filter tuning.** Spin data identified the yaw axis in the IMU frame, and straight-line acceleration data identified the forward axis. The firmware projects the IMU readings onto those axes and removes the stationary forward-acceleration offset. A Kalman filter with fixed gains computed offline from the noise model estimates forward speed, yaw rate, and residual accelerometer bias. The firmware integrates the estimated rates with the trapezoidal rule to obtain distance and heading. [Gain design](kalman_filter_tuning/README.md) · [Calibration procedures](src/TUNING.md)
- **Magnetic feedback.** Experiment 11 corrected SPI read parity and logged field magnitude, shaft angle, and diagnostic status for each AS5047P/AS5147 wheel sensor. Both angles stayed at zero while the quadrature counts advanced. Neither sensor reported ready. All 132 logged wheel updates during the straight legs used quadrature fallback, and the separate IMU compass supplied no valid heading. [Sensor fields and interpretation](src/README.md)

## Comparing runs

Use PlatformIO **Upload and Monitor**, wait for the logging banner, then press the run button. Change one parameter or trajectory feature at a time, repeat the run twice, and compare the same phase in each capture. Keep a change when the target metric improves across repeats and the other phases still meet their targets. Commit that baseline before the next trial. To reproduce the tracking, control-effort, and estimator plots for an individual capture, run this from the repository root. [Logging workflow](src/README.md)

```sh
.venv/bin/python src/scripts/plot_latest_performance.py src/logs/device-monitor-261118-133709.log
```

| Experiment 11 measurement | 13:37:09 run | 13:38:19 run |
| --- | ---: | ---: |
| Estimated forward position at the first stop, target 2 m | 1.949 m | 1.957 m |
| Estimated final distance from the starting point | 3.1 cm | 2.5 cm |
| Estimated final heading relative to the start | −0.53° | −0.68° |

The estimated first waypoint was **5.1 cm short** in one run and **4.3 cm short** in the other. Similar shortfalls on opposite legs can cancel at the finish. For the next comparison, mark the axle midpoint at the first stop as well as the wheel contact centers at the start and finish. Compare the midpoint of the final wheel marks with the midpoint of the starting pair.

## Proposed distance-feedback trial

Use the difference between the original planned distance and `distA`, the trapezoidal integral of estimated forward speed. Both legs are driven forward, so planned cumulative travel reaches 2 m and then 4 m. Feed a bounded correction proportional to this error into the forward-speed reference during straights and pre-turn stops, and disable it during pivots. The four existing controller gains can stay at the experiment 11 settings. Score the first-waypoint error, physical return error, and completion time against this baseline.

Distance feedback, LQR/LQI, and unconstrained linear MPC are proposals for future trials. A scalar LQR on distance error alone gives the same proportional correction. A model that also includes speed-loop lag could address the tradeoff between catching up on a straight and overshooting during braking.
