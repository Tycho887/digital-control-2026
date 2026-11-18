# Turn-rate calibration sequence

The default firmware now runs a complete measurement diagnostic from one
start-button press. Upload with PlatformIO's **Upload and Monitor**, wait for
the logging startup line, place the robot on a clear floor, then press start.
Keep the body still for the first four seconds. No serial commands or prompts
are needed; the robot stops and prints the full capture automatically.

| Time (s) | Wheel-derived yaw reference (rad/s) | Action |
| --- | --- | --- |
| 0–4 | 0 | Motors off; automatically average 1,000 fresh gyro samples and 2,000 fresh acceleration samples. |
| 4–8 | +0.2 | Turn using only the wheel PI controllers. |
| 8–10 | 0 | Motors off; observe coast-down. |
| 10–14 | −0.2 | Turn in the opposite direction. |
| 14–16 | 0 | Motors off. |
| 16–20 | +0.4 | Turn. |
| 20–22 | 0 | Motors off. |
| 22–26 | −0.4 | Turn. |
| 26–28 | 0 | Motors off. |
| 28–32 | +0.6 | Turn. |
| 32–34 | 0 | Motors off. |
| 34–38 | −0.6 | Turn. |
| 38–40 | 0 | Motors off; finish and send the capture. |

Forward reference is zero. Yaw PI, heading correction and the turn-reference
limiter are bypassed, so a gyro error cannot change the wheel commands. The
wheel voltage limit remains 7 V and the gyro scale remains unchanged.
Incomplete stationary calibration, an inconsistent gyro configuration or a
gyro sample gap stops the run and records the reason. Release the button
before another run; an unsent capture is retained until the monitor connects.

Run the existing MATLAB plotting script from the repository root:

```matlab
run('src/scripts/plot_latest_log.m');
```

It selects the newest compatible log, shows **Turn-rate calibration sequence**,
and saves a `_turn_rate_calibration.png` beside it. Its four panels compare
gyro/wheels/Kalman yaw rates, all gyro axes, unwrapped rotation angles, and
wheel PI tracking. The printed table uses phase-boundary angle summaries
computed on the robot before log decimation, and reports wheel/gyro ratios,
actual fresh gyro sample rate and voltage saturation. The raw count conversion
check verifies the active driver's sign and 1,000 deg/s range. Signed fits
across both directions help separate multiplicative mismatch from residual
offset; they do not independently prove a correct wheelbase or gyro scale.

For a text report on the computer:

```sh
.venv/bin/python src/scripts/analyze_turn_rate_calibration.py
```

Both tools select the last turn-rate capture if one monitor file contains
several runs. A normal format-4 trajectory still works with `plot_latest_log.m`.

Repeat the same button sequence with the robot secured upside down and its
wheels clear to compare motor-on and motor-off gyro noise. No firmware change
is needed. The stationary gravity vector records orientation. The wheels turn
while the body stays still, so wheel yaw and fused yaw are not body rotation in
this trial; gyro mean drift or vibration is the useful comparison. Do not use
its wheel/gyro ratio to change the gyro scale or Kalman wheel-noise weights.

The latest floor log, `device-monitor-261114-233349.log`, integrates to about
224° from the encoders versus 95° from the gyro/fused estimate, with about
0.802 m travel. Compare the new floor runs at all three rates in both
directions, and observe the angle externally. A stable factor across rates
suggests a conversion/geometry mismatch; rate-dependent or direction-dependent
differences suggest slip, wheel tracking, or offsets. Accurate straight-line
distance alone cannot validate the wheelbase used for yaw.

The 40 s capture uses 224-byte rows, a divider of 19 (about 52.6 logged rows/s),
and retains the first 55 calibration columns. Columns 56–58 contain hardware
gyro XYZ counts, 59–61 contain gyro angle integrals in radians in the library's
axes, and 62 counts fresh calibrated gyro samples. Integration uses every
fresh sample and its timestamp. `% turn-step` and `% turn-phase` comments
record the schedule and completed phases; the header also records offsets,
gravity, register readback, gains, geometry and completion status. Decimated
noise is useful for comparisons, but is not a complete 1 kHz noise capture.

Select `MainExperiment::OriginalTrajectory` in `src/src/tuning.h` to restore
the original start-button trajectory after diagnosing the measurement.

## Optional controller and estimator tuning

The procedures below use older serial-triggered experiments. They are optional;
the default turn-rate experiment above is fully automatic.

Build/upload the firmware before these trials. Defaults in `src/tuning.h` are
7 V maximum wheel voltage, wheel PI Kp 0.05/0.04 and Ki 0.1, and trial yaw PI
Kp 2.5 and Ki 12.5. Gains use a 1 ms Tustin discretization. Trial gains are
RAM-only; retain successful values in `tuning.h` before the next rebuild.

## 1. Measure yaw before driving

Keep the robot still, send `gyroc`, and wait for the gyro-offset completion
message. Send `cal imu`: it should show `readable=1 valid=1`, a 1000 dps range,
and `calibrated=1`. This checks the active MPU9250_asukiaaa driver; the local
`mpu9250.cpp` is a different driver. The active library negates hardware gyro
values before the mounting sign is applied.

Send `cal stream 100`. Keep still for at least one second, manually rotate
through a measured 360 degrees CCW, keep still for at least one second, then
send `cal stop`. Use one rotation per capture, and repeat clockwise in a
separate capture. Save each monitor log.

```sh
.venv/bin/python src/scripts/analyze_tuning.py rotation CCW.log --angle-deg 360
.venv/bin/python src/scripts/analyze_tuning.py rotation CW.log --angle-deg -360
```

The tool integrates all axes, removes residual offsets using the stationary
ends, and prints candidate `yawAxis`, `yawSign`, and `yawScale` values. Check
repeated rotations in both directions before applying a candidate in
`src/src/tuning.h`. Verify the measured angle within 5% after rebuilding.
After changing these defaults, pass the matching `--axis`, `--sign`, and
`--scale` to the rotation tool when checking the current mapping.

The existing floor logs disagree by about 2.4x in wheel/gyro yaw. That ratio
is supporting evidence, rather than a scale calibration. Compare powered
turns with measured angles too: wheel slip and effective wheelbase affect
wheel-derived yaw while straight-line distance can remain accurate.

## 2. Tune each wheel PI

Send `cal track`, then press start. The 19 s floor trial holds each of
±0.05, ±0.10, ±0.15, and ±0.20 m/s for two seconds, with one-second zero
plateaus before, between directions, and after. It bypasses heading/yaw
control. It travels about one metre forward and back; allow room to stop.
The completed log prints automatically; `cal log` reprints it.

```sh
.venv/bin/python src/scripts/analyze_tuning.py capture TRACK.log
```

Compare each wheel's rise time, overshoot, final error, command variation,
saturation, and encoder-count distance. Change one gain at a time, starting
with proportional gain. Trial commands are accepted only while idle and
disarmed:

```text
cal gains left 0.04 0.1
cal gains right 0.032 0.1
```

These examples reduce Kp by 20%. Keep a change when repeated trials reduce
overshoot/command variation without excessive lag. If final plateau error
exceeds 5%, increase Ki by 20% and repeat; decrease it if slow oscillation or
overshoot worsens. Retain the 7 V limit and existing velocity filters while
comparing gains. `cal stop` cancels an armed or running diagnostic.

## 3. Tune yaw, then restore heading control

After yaw calibration and wheel tuning, send `cal yaw`, then press start.
The 22 s trial alternates +0.2, -0.2, +0.524, and -0.524 rad/s, each held for
three seconds, with two-second zero plateaus. Forward reference is zero.
Heading correction is disabled, calibrated gyro feedback drives the rate PI,
and the existing 0.7 rad/s² limiter stays active. The firmware stops this
trial if gyro configuration, calibration, or freshness is unavailable.

Analyze with `analyze_tuning.py capture YAW.log`. Compare tracking with the
limited reference; the programmed step intentionally becomes a ramp. Target
less than 10% overshoot and less than 5% final plateau error. The output
separates yaw-correction saturation from wheel-voltage saturation.

Use `cal gains yaw 2.5 12.5` to restore the starting trial gains. Adjust Kp
first, then Ki, one at a time in 20% increments. Keep improvements confirmed
by repeated trials. Yaw correction remains bounded at ±0.7 rad/s.

The original trajectory retains heading control. Once the yaw
trial passes, run that trajectory three times and measure distance and total
angle externally: approximately 0.80 m and 90°. Tune the existing
`heading_coeffs` separately if heading correction still introduces overshoot;
do not compensate for a yaw-scale error using controller gains.

## 4. Retune estimator weights from suitable noise captures

Send `cal noise` for three seconds of stationary data and analyze it with
`analyze_tuning.py capture NOISE.log`. For vibration noise with motors running
and the body stationary, use `cal upside` with wheels clear. Wheel-derived
yaw in that trial is not actual body rotation.

Stationary gyro/acceleration sigmas can update their noise inputs. Settled
floor plateaus report wheel-minus-gyro **bias** separately from residual
variation, split by AS5X47 versus encoder source. Moving variation also
contains actual speed/yaw changes; it is not automatically sensor variance.
Only use wheel noise estimates from calibrated, steady, nonslipping motion.

```matlab
addpath('kalman_filter_tuning');
g = solve_dare(struct('sigma_gyro_dps', measuredGyroSigma, ...
    'sigma_accel', measuredAccelSigma, ...
    'sigma_wheel_v', measuredWheelVSigma, ...
    'sigma_wheel_omega', measuredWheelYawSigma));
```

The solver saves generated gains beside itself. Copy the gain block from
`dare_gains.h` into `src/src/kalman.h`, rebuild, and compare tracking lag and
noise on repeated floor trials. Optional `q_v`, `q_om`, and `q_ba` fields tune
process disturbances separately from sensor noise. Select them using these
trial comparisons; a step response alone does not identify all noise terms.

Buffered logs keep their first 55 columns and append hardware gyro counts,
three-axis integrals and a fresh sample count through column 62 (integrals and
the count are populated by the turn-rate sequence). Long trials select a
logging divider to cover their full duration; noise
captures run at 500 Hz. Distinct timestamps exclude repeated measurements,
but decimated data does not contain every 1 kHz estimator update. The stream
preserves its first 19 columns and appends gyro/acceleration timestamps.
