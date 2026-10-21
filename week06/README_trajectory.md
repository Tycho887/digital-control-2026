# Forward–turn–forward identification run

Edit `TrajectoryConfig` defaults in `src/src/trajectory.h`:
0.5 m forward at 0.15 m/s, 90 degrees counterclockwise at 0.50 rad/s,
then 0.5 m forward at 0.15 m/s. Negative turnAngle turns clockwise.
These are commanded amounts: duration = distance/speed or angle/yaw rate.
There is no position feedback, so actual distance and angle may differ.

The sequence includes 4 seconds initially stationary, 0.5 seconds between
motion segments and 1 second at the end. Default total duration is about
15.81 seconds. The firmware checks capture capacity before starting and
aborts before motion if gyro/gravity calibration remains incomplete.
Do not move the robot during initialization/calibration. Verify mounting axes
as described in README_motion_estimator.md. Press the start button once or
send `start` over USB. A held button does not retrigger a completed run.
At completion, motors stop before the serial log is printed automatically.
The robot has not been uploaded or driven by this change.

Record the full serial output to a uniquely named file. PlatformIO's existing
log2file monitor filter can record it (`pio device monitor` from `src`).
Keep header lines: they record trajectory settings and geometry. Columns 1–49
remain compatible with the diagnostic filter log; 50–52 add accelerometer
X/Y/Z in g, 53–55 add driver-calibrated gyro X/Y/Z in degrees/second.
Each log row is 228 bytes; the 400 kB buffer holds 1754 rows (17.54 s).
Longer configured runs are rejected rather than silently truncated.
State column 2 labels baseline/first straight/pause/turn/pause/second straight/
final rest as 10/11/12/13/14/15/16. IMU data is logged at 100 Hz, not its full
native sample rate. Use fusion flags and timestamps when replaying the filter.

In MATLAB:

```matlab
addpath('week06');
experiment = import_trajectory_log('src/logs/my-trajectory.log');
plot(experiment.time, [experiment.commands experiment.state(:,[2 5])]);
```

The importer saves a sibling `my-trajectory-experiment.mat` with raw data,
units documented in field names, phases, commands, estimates and variances.
`experiment.commandSchedule` contains held commands for the robot simulation;
use actual capture timestamps, not nominal tick times, to align responses.
The importer does not overwrite model parameters or run parameter fitting.

Measure final position and heading externally, and preferably position at
segment boundaries. Record floor, battery, load and physical geometry with
the experiment. Repeat runs before tuning. Compare wheel speeds/voltages and
encoder delays to the motor model first, then compare estimator residuals and
external motion to tune filter noise. Filter output alone is not ground truth.
This slow 100 Hz capture is appropriate for chassis and estimator tuning;
it cannot resolve millisecond electrical dynamics for identifying motor L/R.
