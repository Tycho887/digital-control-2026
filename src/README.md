# Firmware and serial logging

Open this directory (`src` in the repository) as the PlatformIO project.
Its source directory is `src/src`, including `basebot_6.ino`.

Use PlatformIO's normal **Upload and Monitor** action. The project hook in
`scripts/wait_for_monitor.py` waits up to 30 seconds for the Teensy's USB serial
port to reappear and remain present for one second after uploading. It only
enumerates USB devices and never opens the serial port. This prevents
PlatformIO's one-time wildcard lookup from missing the rebooting board and
stopping at `Enter port index or full name` before the logger starts.
The hook reports `Teensy USB serial ready: ...` before the monitor opens.
After upload succeeds, the monitor automatically starts recording to `src/logs`. Its
startup output must include `Logging an output to .../src/logs/...` and the
Teensy's `/dev/cu.usbmodem...` port. The project also forces DTR active, which
the Teensy requires to recognize an attached serial reader. Wait for that
startup output, then press the robot's run button once. A completed run is
sent and saved automatically; no `log` command is required.

If the logging startup line never appears, the file writer has not started.
Check the task's upload result and that it is using this `platformio.ini`;
the firmware cannot create a file on the computer without an active monitor.
Only one monitor should own the serial port at a time.
Before repeating the task, terminate the previous monitor task using the
terminal's trash button. Leave the monitor open while executing a run.
The optional **Terminal → Run Task → Upload and Monitor (automatic logging)**
task uses the same combined upload/monitor command and project hook. Close
this custom task before uploading again; PlatformIO's automatic monitor
closure handles its own built-in tasks, but does not handle custom process tasks.

If an already running monitor is stopped at `Enter port index or full name`,
the startup lookup has already failed. Enter the full current robot port
(for example `/dev/cu.usbmodem145870301`) in that existing terminal to connect
without uploading again. Keep the robot powered: a completed capture pending
in RAM is transmitted when the monitor connects. Future combined uploads use
the project hook automatically. A new log file is created when the monitor
connects, even before a robot run; no new file means the file logger has not
started, rather than an empty firmware capture.

PlatformIO's VS Code extension installs its command-line tools in
`~/.platformio/penv/bin`, which may not be on your terminal's `PATH`.
No separate installation is needed. For an optional monitor-only session,
this command works from any directory:

```sh
"$HOME/.platformio/penv/bin/platformio" run \
  --project-dir "$HOME/Autumn2026/digital-control-2026/src" \
  --target monitor --environment teensy41
```

Close an existing monitor before starting another. The normal VS Code
**Upload and Monitor** action already uses the installed executable directly;
it does not require `pio` to be available in your shell. The `pio` shorthand
in the diagnostic commands below refers to `~/.platformio/penv/bin/pio`.

The configured monitor port matches `/dev/cu.usbmodem*` on macOS, excluding
Bluetooth devices and the debug console. Check the port in the monitor's
startup banner. If multiple USB serial devices are connected, use
`pio device list` and pass the Teensy's port explicitly with `--port`.
The Teensy has USB vendor/product ID `16C0:0483`.

For diagnostics, `log` plus Enter requests the samples already held in RAM;
it does not start the motors. Even with no samples, the robot prints a header
and `% sent 0 log lines`. The current firmware prints `% log format 3` and does
not log a covariance. Columns 1-41 retain their format-2 meanings: columns
27-28 are estimated linear/angular velocity, column 29 is magnetometer heading,
columns 30-31 are wheel-speed setpoints, column 32 is yaw-rate reference,
columns 33-37 are `[v, omega, theta, b_a, b_g]`, columns 38-39 are
reference-versus-estimate errors, and columns 40-41 are outer-loop commands.

Format 3 appends filtered wheel-PI feedback (42-43), independent AS5X47 wheel
velocities (44-45), desired heading and actual heading-controller error
(46-47), limited turn reference and heading correction (48-49), estimator
acceleration/gyro inputs (50-51), four innovations (52-55), the KF update mask
(56), and diagnostic flags (57). The flags record KF feedback/health, AS5X47
status, encoder fallback, and wheel/heading/yaw-controller saturation. The
header also records the controller coefficients, reference limiters, trajectory
parameters, geometry, log divider, and whether KF feedback was enabled.

The main loop currently selects `upsideDownCalibrationSequence()`. Keep the
monitor open, secure the robot upside down with the wheels clear, and press start
once. The firmware initializes a 500 Hz calibration capture, runs the six-second
schedule below, stops the motors, and sends one `% CAL upside` capture. It does
not emit the standard format-3 header.

To capture the upright driving run instead, comment
`upsideDownCalibrationSequence()` in `loop()` and uncomment
`closedLoopTrajectory()`. That sequence has a 0.2 s stationary baseline,
commands 0.20 m/s forward for 4.0 s, then +30 degrees/s (CCW) for 3.0 s with
zero forward reference, followed by 0.4 s settling. The heading reference
advances during the turn so the heading controller does not oppose it. The
complete 7.6 s run is logged at 250 Hz (`logDivider = 4`) as one format-3
capture. The 216-byte rows fit the shared 495 kB buffer with margin; a
compile-time check requires 8 s of coverage. Older experiments remain available
by commenting the selected call and uncommenting `sequenceTwoSteps()`,
`yawRateStep()`, or `forwardThreeSeconds()`.
The firmware applies zero PWM before printing the completed log. It waits
for a connected USB serial monitor before automatically sending a completed
capture, so opening the monitor after the run can still retrieve the output.
Wait for `% sent ... log lines` before closing the
monitor with Ctrl+C. Output is saved to `logs/device-monitor-YYMMDD-HHMMSS.log`.
If a run finished before the monitor opened, a pending capture is sent when
the monitor connects. `log` can optionally repeat a transfer while the robot
remains powered; rebooting or starting another run replaces the samples.
Requests during a run are rejected so serial printing cannot stall motor
control. A buffer allocation failure prevents starting a run; if a future
sequence exceeds capacity, its log includes a warning about missing samples.

The pose and velocity columns come from the stationary decoupled estimator in
`src/src/kalman.h` and `kalman.cpp`. It runs at 1 kHz with state
`[v, b_a, theta, omega, b_g]`: forward acceleration drives `v`, wheel `v`
corrects `[v, b_a]`, and gyro / wheel `omega` / gated magnetometer heading
correct `[theta, omega, b_g]`. The gains are constant; no covariance is
propagated. Distance, `x`, and `y` are dead-reckoned outside the filter from
`v` and `theta`, so wheel slip cannot inject an absolute pose error. The stock
48-edge quadrature encoders still feed only the inner wheel-speed PI.
Columns 11-14 report pose, columns 27-28 linear and angular velocity, column 29
the magnetometer heading (`NaN` until a valid fix), and columns 33-37 the
estimator state.

The IMU driver enables the AK8963; `magc` runs a hard-iron calibration (rotate
the robot 360 degrees by hand for 8 seconds), and `mag 0` / `mag 1` disable or
enable the magnetometer heading. Mag offsets are RAM-only and printed after
calibration; reboot after calibrating so the estimator starts from the
calibrated heading.

The control path uses reference limiters for forward speed and turn rate, a
heading lead-lag controller, yaw-rate PI, and the per-wheel encoder PI. The
current wheel coefficients come from `Motor_model_param.m` (`Kp = 0.05/0.04`,
`Ki = 0.1` at 1 ms), with a conservative 3 V actuator limit. There is no outer
forward-velocity PID in this architecture. `useKfFeedback` near the top of
`basebot_6.ino` defaults to `true`; setting it to `false` bypasses only the KF
heading/yaw feedback, leaving reference-limited feed-forward and the inner
encoder loops active. The KF continues running and logging in both modes, and
the selected mode is printed in each log header.

## Calibration experiments

Three USB-triggered experiments share a compact raw capture and the same
495 kB buffer. A standard run and a calibration capture are mutually
exclusive: starting one discards an unread capture from the other.

| Command | Motors | Capture | Procedure |
| --- | --- | --- | --- |
| `cal mag` | off | 200 Hz, 15 s | Rotate the robot 360 degrees by hand and back. |
| `cal noise` | off | 1 kHz, 3 s | Hold the robot still. |
| `cal track` | PI | 500 Hz, 6 s | Clear the floor, then press the start button (or send `start`). |
| `cal upside` | wheel PI | 500 Hz, 6 s | Secure the robot upside down with wheels clear, then press start. |
| `cal stop` | off | - | Stop now and keep the samples captured so far. |
| `cal log` | off | - | Re-send a completed calibration capture. |
| `cal retrymag` | off | - | Re-run AK8963 bring-up and print WHO_AM_I, ST1, ST2 and raw values. |

`cal mag` and `cal noise` start immediately and print their instructions.
`cal track` and `cal upside` arm first. `cal track` runs the step schedule
0, +0.05, +0.10, +0.15,
0, -0.05, -0.10, -0.15, 0 m/s with 0.3-0.8 s segments. The firmware sends a
completed capture automatically when the monitor is connected; `cal log`
repeats it while the robot stays powered.

`upsideDownCalibrationSequence()` starts with 0.5 s at zero reference, then
spins both wheels at the equivalent of 0.20 m/s for 2.5 s, applies differential
wheel speeds corresponding to +30 degrees/s for 2.5 s, and finishes with 0.5 s
at zero before forcing the motors off. It uses only the inner encoder PI for
actuation while the estimator and raw-sensor logging continue. This supports
motor-on noise and wheel-loop diagnostics without chassis motion; wheel-derived
motion is not chassis ground truth in this configuration. When selected in
`loop()`, this function waits for the start button and initializes the capture
itself. The USB command `cal upside` arms the same schedule for a start-button
press. Do not call the low-level phase stepper directly from `loop()`; doing so
bypasses initialization and completion handling.

The calibration and driving procedures are separate. Save the completed upside
capture before starting another experiment because all captures share one RAM
buffer. Then select `closedLoopTrajectory()` in `loop()`, place the robot upright
with clear floor space, rebuild/upload, and press start. For a feed-forward
comparison, set `useKfFeedback = false` and repeat the upright trajectory.

The capture header is `% CAL <mode>` followed by 40 original columns plus
column 41, the sequence phase: time, mode,
sensor-fresh flags, estimator-update mask, setpoints, PI feedback, motor
voltage and encoder velocity, raw gyro/accel/mag, AS5X47 angle and velocity,
wheel window, mag heading, estimator `v/omega/theta/b_a/b_g`, and the last
innovation `y` plus a zero variance column for the gyro, wheel-v, wheel-omega
and mag updates. `gyro` is deg/s, `acc` is g, and `mag` is in the library's
adjusted counts (0.15 uT/LSB). For upside captures, phase 10 is the baseline,
11 is forward wheel motion, 12 is differential turning, and 2 is final settling.
The zero variance columns are intentional: the stationary filter does not
propagate `S`. Noise captures provide measurements for later gain retuning;
there is no on-device `R` matrix to update automatically.

Analyze the newest existing mag/noise/track capture with:

```sh
python3 src/scripts/analyze_calibration.py
python3 src/scripts/analyze_calibration.py src/logs/device-monitor-YYMMDD-HHMMSS.log
```

The script auto-detects the mode. `mag` reports the accelerometer gravity axis,
the field axis with the smallest rotation variation, and the best heading
pair/sign against gyro-integrated yaw (including the firmware's current
`atan2(Y, -Z)`). `noise` reports measured 1-sigma values against the filter's
assumed values; NIS is unavailable because the stationary gains do not
propagate `S`. `track` reports rise time, overshoot, settling time,
steady-state and RMS error, and 3 V saturation for the PI feedback, AS5X47
velocity and estimator velocity.
Pass `--outdir DIR` to save PNGs instead of opening windows, or `--no-plots`
for the table only. Upside-specific and format-3-specific analysis is deferred
until real captures are available; the serial headers are the authoritative
column descriptions for those new logs.

If column 29 of a standard log is `NaN` throughout, run `cal retrymag` and
check the printed ST1/ST2 values. `cal mag` also reports whether the
magnetometer was available, so an axis check is not confused with a missing
sensor.

For manual builds and uploads, use PlatformIO:

```sh
pio run -e teensy41 -t clean
pio run -e teensy41
pio run -e teensy41 -t upload
```

The legacy `compile` and `upload` scripts target the old Arduino build in
`build`, whose HEX file can be stale. Use the PlatformIO upload command to
flash `.pio/build/teensy41/firmware.hex`.
