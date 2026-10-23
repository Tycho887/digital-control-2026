# Firmware and serial logging

Open this directory (`src` in the repository) as the PlatformIO project.
Its source directory is `src/src`, including `basebot_6.ino`.

Use **Terminal → Run Task → Upload and Monitor (automatic logging)** in VS
Code. This saved task uploads the current `teensy41` firmware, then opens a
monitor with the project path, environment, USB port, DTR, and `log2file`
filter explicitly selected. After upload succeeds, the monitor automatically
starts recording to `src/logs`. Its
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
The built-in PlatformIO **Upload and Monitor** action should also inherit the
same project settings; the saved task makes the recording options explicit.

For a monitor-only session, the equivalent terminal command from this
directory is:

```sh
pio device monitor -e teensy41
```

The configured monitor port matches `/dev/cu.usbmodem*` on macOS, excluding
Bluetooth devices and the debug console. Check the port in the monitor's
startup banner. If multiple USB serial devices are connected, use
`pio device list` and pass the Teensy's port explicitly with `--port`.
The Teensy has USB vendor/product ID `16C0:0483`.

For diagnostics, `log` plus Enter requests the samples already held in RAM;
it does not start the motors. Even with no samples, the robot prints a header
and `% sent 0 log lines`. The current firmware prints
`% 25-26 computed control signal (left,right) (V)` in the header.

To capture a new run, keep the monitor open and press the robot's start button.
The current forward sequence takes about 3.6 seconds (0.2 seconds at rest,
3 seconds driving at 0.02 m/s, and 0.4 seconds settling). The 420 kB buffer
holds 3750 samples at 1 ms with the current 112-byte rows, covering the full run.
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

For manual builds and uploads, use PlatformIO:

```sh
pio run -e teensy41 -t clean
pio run -e teensy41
pio run -e teensy41 -t upload
```

The legacy `compile` and `upload` scripts target the old Arduino build in
`build`, whose HEX file can be stale. Use the PlatformIO upload command to
flash `.pio/build/teensy41/firmware.hex`.
