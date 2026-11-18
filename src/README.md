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

The default experiment is **Turn-rate calibration sequence**. Keep the robot
still for four seconds after pressing start: gyro and forward-acceleration
offsets are calibrated automatically. It then performs six four-second turns
at +0.2, −0.2, +0.4, −0.4, +0.6 and −0.6 rad/s, with two seconds of motors-off
coast-down after each turn. The complete run lasts 40 seconds. Only the wheel
PI controllers drive the motors; yaw/heading feedback and reference limiters
are bypassed. [The sequence and interpretation are documented here](TUNING.md).

Run `src/scripts/plot_latest_log.m` in MATLAB from the repository root. It
automatically recognizes the new capture, plots gyro/wheel/fused yaw,
three-axis gyro rates and integrated angles, and wheel tracking, and prints
per-turn ratios and actual gyro sample rates. It saves a PNG beside the log.
Repeat the same button sequence with the body secured upside down and wheels
clear to compare motor vibration with the stationary gyro. Wheel-derived yaw
is not body rotation in that orientation. No device commands or firmware
change are needed for either run.

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
and `% sent 0 log lines`. The current firmware prints `% log format 4` and does
not log a covariance. Columns 1-41 keep their format-2 positions; the estimator
and innovation meanings changed: columns
27-28 are estimated linear/angular velocity, column 29 is magnetometer heading,
columns 30-31 are wheel-speed setpoints, column 32 is yaw-rate reference,
columns 33-37 are `[v, omega, dead-reckoned theta, b_a, 0]`, columns 38-39 are
the actual control errors (`vCmd - v` and the yaw-rate PID error), and columns
40-41 are outer-loop commands.

Format 4 appends filtered wheel-PI feedback (42-43), independent AS5X47 wheel
velocities (44-45), desired heading and actual heading-controller error
(46-47), limited turn reference and heading correction (48-49), estimator
acceleration/gyro inputs (50-51), three innovations plus a reserved slot
(52-55), the KF update mask (56), and diagnostic flags (57). The flags record KF feedback/health, AS5X47
status, encoder fallback, and wheel/heading/yaw-controller saturation. The
header also records the controller coefficients, reference limiters, trajectory
parameters, geometry, log divider, and whether KF feedback was enabled.

`MainExperiment::TurnRateCalibration` in `src/src/tuning.h` selects the default
button sequence. To restore the original controller trajectory, select
`MainExperiment::OriginalTrajectory` and rebuild/upload. Place the robot
upright with clear floor space, keep the monitor open, and press start once.
The run starts on a new button press and the button must be released before
another run; the input uses the internal pull-down, so a held or floating
start input cannot restart the sequence after the log is sent.
The 7.6 s run has a 0.2 s stationary baseline, 4.0 s at 0.20 m/s forward,
3.0 s at +30 degrees/s (CCW), and 0.4 s settling. The heading reference
advances during the turn so the heading controller does not oppose it. The
complete run is logged at 250 Hz (`logDivider = 4`) as one format-4 capture.
The 216-byte rows fit the shared 495 kB buffer with margin; a compile-time
check requires 8 s of coverage.

The older six-second wheels-off experiment remains available through
`upsideDownCalibrationSequence()` or `cal upside`; it sends `% CAL upside`
instead of the standard format-4 header. The default 40-second sequence can
also run upside down without changing the selected mode. Other older
experiments remain available in `loop()` as commented calls.
The firmware applies zero PWM before printing the completed log. During a
standard run it also streams a 20 Hz status line, prefixed `% live ...`. The
live rows are comments for the format-4 analyzer, but they show time, state,
estimator `v`/`omega`, heading, outer-loop commands, wheel setpoints and the
heading/yaw errors in the monitor. They are written only when the USB transmit
buffer has room, so a missing or stalled monitor cannot stall the 1 kHz loop.

The full 250 Hz capture is sent when a DTR-asserting monitor is connected.
Wait for `% sent ... log lines` before closing the monitor. If no monitor is
connected at the end of the run, the capture is retained and a new start is
refused until the log is retrieved. `log` forces a transfer (and also works
when the monitor does not assert DTR); `log clear` discards the capture
deliberately. `status` prints the run state, sample count, pending flags,
USB/DTR state and free heap. Rebooting or power-cycling still clears RAM, so
retrieve the capture or rely on the live rows before resetting.

Requests during a run are rejected so serial printing cannot stall motor
control. A buffer allocation failure prevents starting a run; if a future
sequence exceeds capacity, its log includes a warning about missing samples.

The pose and velocity columns come from the stationary decoupled estimator in
`src/src/kalman.h` and `kalman.cpp`. It runs at 1 kHz with state
`[v, omega, b_a]`: forward acceleration drives `v`, wheel `v` corrects
`[v, b_a]`, and the gyro / wheel `omega` correct `omega`. Heading `theta` and
distance `x`, `y`, `s` are dead-reckoned outside the filter from `v` and
`omega` with the trapezoidal rule. The gains are constant; no covariance is
propagated. The stock 48-edge quadrature encoders feed both the filter and the
inner wheel-speed PI. Columns 11-14 report pose, columns 27-28 linear and
angular velocity, column 29 the raw magnetometer heading (diagnostic only,
`NaN` when unavailable), and columns 33-37 `[v, omega, theta, b_a, 0]`.

The IMU driver brings up the AK8963 behind the MPU-9250's bypass path. The
whole IMU bus runs at 400 kHz, the MPU-9250 and AK8963 limit; the old 1 MHz
setting is gone. The driver reads ST1 (0x02) explicitly before each 7-byte
field read, because the library's `magUpdate()` starts at HXL (0x03), and it
retries a data-ready sample instead of latching the sensor off after one
missed sample. `cal retrymag` prints the MPU and AK WHO_AM_I, CNTL1, BYPASS_EN,
I2C-master state, ST1/ST2, ASA values, offsets and raw counts; `cal mag` prints
the same probe before its 15 s capture. `who=0x48` is a responding AK8963;
`mpu=0x70` is an MPU-6500 with no magnetometer. `magc` runs a hard-iron
calibration (rotate the robot 360 degrees by hand for 8 seconds), and `mag 0` /
`mag 1` disable or enable the diagnostic heading. The estimator does not use
the magnetometer: it is broken/unresponsive on the current hardware, and the
heading is dead-reckoned from the gyro. Mag offsets are RAM-only and printed
after calibration.

The control path uses reference limiters for forward speed and turn rate, a
heading lead-lag controller, yaw-rate PI, and the per-wheel encoder PI. The
current wheel coefficients come from `Motor_model_param.m` (`Kp = 0.05/0.04`,
`Ki = 0.1` at 1 ms), with a 7 V actuator limit. Trial yaw PI gains are
`Kp = 2.5`, `Ki = 12.5`. [Follow the tuning sequence](TUNING.md) for measured
yaw calibration and isolated wheel/yaw trials. There is no outer
forward-velocity PID in this architecture. `useKfFeedback` near the top of
`basebot_6.ino` defaults to `true`; setting it to `false` bypasses only the KF
heading/yaw feedback, leaving reference-limited feed-forward and the inner
encoder loops active. The KF continues running and logging in both modes, and
the selected mode is printed in each log header.

## Calibration experiments

The automatic turn-rate sequence and optional USB-triggered experiments share
a compact raw capture and the same 495 kB buffer, plus an unbuffered stream.
The USB commands below are not required for the default sequence.
A standard run and a calibration
capture are mutually exclusive; send or clear a pending capture before
starting another.

| Command | Motors | Capture | Procedure |
| --- | --- | --- | --- |
| Start button (default mode) | wheel PI | ~52.6 Hz, 40 s | Keep still for 4 s; automatic bidirectional turns and stops follow. |
| `cal mag` | off | ~143 Hz, 15 s | Rotate the robot 360 degrees by hand and back. |
| `cal noise` | off | 500 Hz, 3 s | Hold the robot still. |
| `cal track` | wheel PI | ~111 Hz, 19 s | Clear the floor, then press the start button (or send `start`). |
| `cal yaw` | wheel + yaw PI | 100 Hz, 22 s | Calibrate yaw first, then press start. Heading disabled. |
| `cal upside` | wheel PI | ~333 Hz, 6 s | Secure the robot upside down with wheels clear, then press start. |
| `cal imu` | unchanged | - | Read back gyro range, DLPF, divider and calibration status. |
| `cal gains left\|right\|yaw Kp Ki` | off | - | Apply RAM-only PI trial gains while idle and disarmed. |
| `cal stream [Hz]` | off | 50-500 Hz, 20 min | Keep still for noise or manually rotate for calibration; end with `cal stop`. |
| `cal stop` | off | - | Stop now and keep the samples captured so far. |
| `cal log` | off | - | Re-send a completed calibration capture. |
| `cal clear` | off | - | Discard a completed calibration capture. |
| `status` | off | - | Print run state, sample/pending flags, USB/DTR and free heap. |
| `log` | off | - | Re-send the standard format-4 capture (forces the transfer). |
| `log clear` | off | - | Discard the pending standard capture. |
| `cal retrymag` | off | - | Probe the AK8963 and print WHO_AM_I, bypass, CNTL1, ST1/ST2 and raw values. |

`cal mag` and `cal noise` start immediately and print their instructions.
`cal track`, `cal yaw`, and `cal upside` arm first. `cal track` runs
0, +0.05, +0.10, +0.15, +0.20, 0, -0.05, -0.10, -0.15, -0.20, 0 m/s,
with two-second moving plateaus and one-second zero plateaus (19 s total).
`cal yaw` isolates the yaw-rate PI with calibrated gyro feedback, no heading
correction, and zero forward reference for 22 s. `cal gains left|right|yaw Kp Ki`
changes trial gains while idle; `cal imu` checks gyro configuration readback.
The firmware sends a
completed capture automatically when the monitor is connected; `cal log`
repeats it while the robot stays powered. A pending capture is retained until
it is sent or discarded with `cal clear`, and a new run is refused meanwhile.

`cal stream` is unbuffered: it prints one row per divider at 50-500 Hz for up
to 20 minutes and stops with `cal stop`, or automatically when a run or a
buffered capture starts. Use it for the 10-minute stationary logs that the
Allan analysis needs. Keep the monitor open and the robot still; if the host
stops draining USB, the print can stall the 1 kHz control loop, so do not
drive while streaming.

`upsideDownCalibrationSequence()` starts with 0.5 s at zero reference, then
spins both wheels at the equivalent of 0.20 m/s for 2.5 s, applies differential
wheel speeds corresponding to +30 degrees/s for 2.5 s, and finishes with 0.5 s
at zero before forcing the motors off. It uses only the inner encoder PI for
actuation while the estimator and raw-sensor logging continue. This supports
motor-on noise and wheel-loop diagnostics without chassis motion; wheel-derived
motion is not chassis ground truth in this configuration. The wheels rotate
without the body turning, so the wheel-omega measurement disagrees with the
gyro. The estimator has no `b_g` state; treat that disagreement as an
experiment artifact, not as a calibration result. When
selected in `loop()`, this function waits for the start button and initializes
the capture itself. The USB command `cal upside` arms the same schedule for a
start-button press. Do not call the low-level phase stepper directly from
`loop()`; doing so bypasses initialization and completion handling.

The calibration and driving procedures are separate. A standard ground run and
a calibration capture share one RAM buffer, so save the completed capture
before starting the other. The turn-rate sequence is selected by default in
`tuning.h`. For a feed-forward comparison of the original trajectory, select
`MainExperiment::OriginalTrajectory`, set `useKfFeedback = false` and repeat it
upright after rebuilding/uploading.

The capture header is `% CAL <mode>` followed by 40 original columns, column
41 sequence phase, and columns 42-45: AK8963 status (low byte ST1, high byte
ST2), IMU sample interval (us), and the low 16 bits of the encoder counts.
Columns 46-55 append yaw reference/limited reference/command, mapped gyro
input, actual wheel-update inputs, gyro/wheel/acceleration timestamps, and
source/saturation flags. The logging divider covers the complete longer
trials; stationary noise uses 500 Hz. Rows now use 224 bytes and append hardware
gyro XYZ counts in columns 56–58, three-axis unwrapped gyro integrals in radians
in 59–61, and cumulative fresh calibrated gyro sample count in 62. The last
four fields are populated for the turn-rate sequence; each gyro integral uses
every fresh update before decimation. Its `% turn-phase` comments preserve
phase-boundary angles and sample counts, and the header records the automatic
calibration, gravity and gyro-register readback. Flags 11–13 identify completed
gyro calibration, completed acceleration calibration and valid gyro
configuration. `analyze_tuning.py capture LOG` handles
these timestamps and encoder count wrapping.
The original columns are time, mode,
sensor-fresh flags, estimator-update mask, setpoints, PI feedback, motor
voltage and encoder velocity, raw gyro/accel/mag, AS5X47 angle and velocity,
wheel window, mag heading, estimator `v/omega/theta/b_a/reserved`, and the last
innovation `y` plus a zero variance column for the gyro, wheel-v, wheel-omega
and reserved slots. `gyro` is deg/s, `acc` is g, and `mag` is in the library's
adjusted counts (0.15 uT/LSB). The mag columns and flags 8-10 (attempt, DRDY,
overflow) are diagnostic only; the estimator does not consume them. For upside captures, phase 10 is the baseline,
11 is forward wheel motion, 12 is differential turning, and 2 is final settling.
The zero variance columns are intentional: the stationary filter does not
propagate `S`. Noise captures provide measurements for later gain retuning;
there is no on-device `R` matrix to update automatically.

Analyze the newest turn-rate capture with MATLAB's `plot_latest_log.m` or
`python3 src/scripts/analyze_turn_rate_calibration.py`. Analyze older
mag/noise/track/yaw/upside captures with:

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
steady-state and RMS error, and saturation at the recorded voltage limit for the PI feedback, AS5X47
velocity and estimator velocity. `upside` reports sensor health (dead AS5X47
bus, magnetometer availability, encoder-fallback share, zero-velocity feed
events), per-phase gyro and accelerometer noise, the estimator's first-order
velocity response and steady-state tracking against the wheel reference, and
the bias/heading behaviour. Wheel-derived velocity is not chassis ground truth
while the robot is upside down, so the report keeps those quantities separate.
Pass `--outdir DIR` to save PNGs instead of opening windows, or `--no-plots`
for the table only.

A long `cal stream` capture is analyzed with:

```sh
python3 src/scripts/analyze_stream.py
python3 src/scripts/analyze_stream.py src/logs/device-monitor-YYMMDD-HHMMSS.log
```

It reports the actual sample rate, per-axis noise, the mag attempt/DRDY/
overflow counts, and the overlapping Allan deviation for gyro yaw and accel
forward. The white-noise density (`q`) and the ADEV minimum are the seeds for
`q_ba` in the DARE; the per-sample sigma is the measurement-noise input.
Pass `--outdir DIR` to save the Allan plot or `--no-plots` for the table only.
The stream is stationary data; do not use it while driving.

If column 29 of a standard log is `NaN` throughout, run `cal retrymag`. A
`who=0x48` answer means the AK8963 is responding; `who=0x00` with `mpu=0x71`
points at the bypass path or PCB, while `mpu=0x70` is an MPU-6500 without a
magnetometer. `cal mag` prints the same probe before its capture, so an axis
check is not confused with a missing sensor.

For manual builds and uploads, use PlatformIO:

```sh
pio run -e teensy41 -t clean
pio run -e teensy41
pio run -e teensy41 -t upload
```

The legacy `compile` and `upload` scripts target the old Arduino build in
`build`, whose HEX file can be stale. Use the PlatformIO upload command to
flash `.pio/build/teensy41/firmware.hex`.
