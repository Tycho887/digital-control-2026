# Firmware and serial logging

Run PlatformIO commands from this directory (`src` in the repository). Its
source directory is `src/src`, including `basebot_6.ino`.

```sh
pio device monitor -e teensy41
```

The configured monitor port matches `/dev/cu.usbmodem*` on macOS, excluding
Bluetooth devices and the debug console. Check the port in the monitor's
startup banner. If multiple USB serial devices are connected, use
`pio device list` and pass the Teensy's port explicitly with `--port`.
The Teensy has USB vendor/product ID `16C0:0483`.

With the monitor open, type `log` and press Enter. This requests the samples
already held in RAM; it does not start the motors. Even with no samples, the
robot prints a header and `% sent 0 log lines`. The current firmware prints
`% 25-26 computed control signal (left,right) (V)` in the header.

To capture a new run, keep the monitor open and press the robot's start button.
The firmware records during the sequence and automatically prints the log
after about 3.1 seconds. Wait for `% sent ... log lines` before closing the
monitor with Ctrl+C. Output is saved to `logs/device-monitor-YYMMDD-HHMMSS.log`.
If a run finished before the monitor opened, send `log` to retrieve it while
the robot remains powered; rebooting or starting another run replaces it.

For manual builds and uploads, use PlatformIO:

```sh
pio run -e teensy41 -t clean
pio run -e teensy41
pio run -e teensy41 -t upload
```

The legacy `compile` and `upload` scripts target the old Arduino build in
`build`, whose HEX file can be stale. Use the PlatformIO upload command to
flash `.pio/build/teensy41/firmware.hex`.
