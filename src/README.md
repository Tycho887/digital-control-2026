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
and `% sent 0 log lines`. The current firmware prints
`% 25-26 computed control signal (left,right) (V)` in the header; columns 27-28
are the Kalman filter's linear and angular velocity (m/s, rad/s), column 29 is
the magnetometer heading (rad), columns 30-31 are the wheel-speed controller
setpoints (left, right, motor rad/s), columns 32-38 are the Kalman state
`[s, theta, v, omega, b_a, b_g, b_m]`, and columns 39-66 are the lower triangle
of the filter covariance `P`, stored row-major as
`P00; P10 P11; P20 P21 P22; ...; P66`. Because `P` is symmetric, the lower
triangle is the full matrix.

To capture a new run, keep the monitor open and press the robot's start button.
The active `sequenceTwoSteps` run lasts about 1.8 seconds. The 495 kB buffer
holds 1903 samples at 1 ms with the current 260-byte rows, covering about
1.9 seconds. For the 3.6-second `forwardThreeSeconds` sequence, set
`logDivider = 2` (500 Hz) and `logCoverageUs = 3700000` in `basebot_6.ino`; the
buffer then covers about 3.8 seconds.
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

The pose and velocity columns come from the 7-state Kalman filter in
`src/src/kalman.h` and `kalman.cpp`. It runs at 1 kHz and fuses forward
acceleration (as a control input), gyro yaw rate, linear and angular velocity
derived from the two AS5047P/AS5147 absolute-angle sensors over SPI (angle
differences over a 10 ms window), and a gated magnetometer heading. The stock
48-edge quadrature encoders no longer feed the filter: they now only provide
feedback to the wheel-speed controller. Distance and heading are integrated
from the filter's own velocity and turn rate, so wheel slip cannot inject an
absolute pose error; the heading stays anchored by the gyro and magnetometer.
Columns 11-14 report its x, y, heading and distance, columns 27-28 its linear
and angular velocity, columns 32-38 its state and columns 39-66 its covariance,
and column 29 the magnetometer heading, which prints
`NaN` until the magnetometer has a valid fix. The IMU driver enables the
AK8963; `magc` runs a hard-iron calibration (rotate the robot 360 degrees by
hand for 8 seconds), and `mag 0` / `mag 1` disable or enable the magnetometer
heading. Mag offsets are RAM-only and printed after calibration; reboot after
calibrating so the filter starts from the calibrated heading. The `LogData`
row is 260 bytes and the 495 kB buffer holds 1903 samples.

The wheel-speed PI now uses the week07-tuned difference equations in
`left_wheel_coeffs` / `right_wheel_coeffs` (`Kp ~= 0.27`, `Ki ~= 3.9-5.8` at
1 ms), with a named 3 V actuator limit. `sequenceTwoSteps` commands 0.05 and
0.10 m/s; the older 1.5/3 m/s commands were far beyond what the 3 V limit can
reach.

## Calibration experiments

Three USB-triggered experiments share a compact raw capture and the same
495 kB buffer. A standard run and a calibration capture are mutually
exclusive: starting one discards an unread capture from the other.

| Command | Motors | Capture | Procedure |
| --- | --- | --- | --- |
| `cal mag` | off | 200 Hz, 15 s | Rotate the robot 360 degrees by hand and back. |
| `cal noise` | off | 1 kHz, 3 s | Hold the robot still. |
| `cal track` | PI | 500 Hz, 6 s | Clear the floor, then press the start button (or send `start`). |
| `cal stop` | off | - | Stop now and keep the samples captured so far. |
| `cal log` | off | - | Re-send a completed calibration capture. |
| `cal retrymag` | off | - | Re-run AK8963 bring-up and print WHO_AM_I, ST1, ST2 and raw values. |

`cal mag` and `cal noise` start immediately and print their instructions.
`cal track` arms first; it runs the step schedule 0, +0.05, +0.10, +0.15,
0, -0.05, -0.10, -0.15, 0 m/s with 0.3-0.8 s segments. The firmware sends a
completed capture automatically when the monitor is connected; `cal log`
repeats it while the robot stays powered.

The capture header is `% CAL <mode>` followed by 38 columns: time, mode,
sensor-fresh flags, KF-update mask, setpoints, PI feedback, motor voltage and
encoder velocity, raw gyro/accel/mag, AS5X47 angle and velocity, wheel window,
mag heading, KF `v/omega/theta`, and the last innovation `y` and `S` for the
gyro, wheel-v, wheel-omega and mag updates. `gyro` is deg/s, `acc` is g, and
`mag` is in the library's adjusted counts (0.15 uT/LSB).

Analyze the newest capture with:

```sh
python3 src/scripts/analyze_calibration.py
python3 src/scripts/analyze_calibration.py src/logs/device-monitor-YYMMDD-HHMMSS.log
```

The script auto-detects the mode. `mag` reports the accelerometer gravity axis,
the field axis with the smallest rotation variation, and the best heading
pair/sign against gyro-integrated yaw (including the firmware's current
`atan2(Y, -Z)`). `noise` reports measured 1-sigma values against the filter's
assumed values and the mean normalized innovation squared (NIS) per update.
`track` reports rise time, overshoot, settling time, steady-state and RMS
error, and 3 V saturation for the PI feedback, AS5X47 velocity and KF velocity.
Pass `--outdir DIR` to save PNGs instead of opening windows, or `--no-plots`
for the table only.

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
