# Diagnostic Kalman motion estimator

The Teensy runs an eight-state, fixed-size float Kalman filter at 100 Hz using
BasicLinearAlgebra 5.1. State order is signed travel, forward velocity,
forward acceleration, unwrapped heading, yaw rate, yaw acceleration,
residual acceleration bias, residual gyro bias. Predictions use constant
acceleration with white-jerk process noise. Independent scalar updates use
Joseph covariance updates; no inversion or dynamic allocation is needed.
The motor controller continues to use its original encoder feedback.

## Before a robot experiment

Verify the mounting constants in `basebot_6.ino`: forward=-IMU Z and
counterclockwise yaw=-IMU X are inferred from existing tilt code, not measured.
Check a forward push and a counterclockwise rotation. Confirm the configured
gear ratio 9.6, radius 0.0315 m and track width 0.14 m physically.
Keep the robot still for gyro calibration, followed by 200 fresh estimator-rate
acceleration samples (about two seconds). Gravity calibration is indicated by
flag 16. Longitudinal acceleration uses a stationary gravity reference, valid
on a flat floor with fixed pitch; slopes or chassis rocking contaminate it.
The gyro bias state estimates residual bias after the existing driver offset.
Reboot to repeat gravity calibration. Starting a capture resets estimated
motion and covariance, retaining the gravity reference.

The IMU interface checks data-ready status and exposes successful fresh read
timestamps. The estimator consumes only fresh readings within 20 ms. Encoder
updates use forward-positive motor speed only when counts change, with a
noise floor and delay-dependent variance. After 150 ms without an edge a
noisy zero-speed observation bounds drift; this is a low-speed approximation,
not an independent stationary detector. It can underestimate creeping motion.
Noise parameters are initial tuning values, not calibrated specifications.
Encoder delay is approximated through measurement noise, not compensated by
replaying past states. No sensor observations means prediction only. A gap
above 100 ms marks health false until the next capture reset.

## Logs

Existing columns 1–24 retain their meanings. Columns appended:

| Columns | Meaning |
|---|---|
| 25–32 | Eight states in the order above, SI units |
| 33–40 | Corresponding covariance diagonals, SI units squared |
| 41–42 | Derived X/Y, midpoint integration of filtered travel/heading increments |
| 43–44 | Latest acceleration and gyro values used, SI; NaN before first use |
| 45–46 | Commanded forward velocity and yaw rate |
| 47 | Estimator timestamp, microseconds since capture start |
| 48 | Estimator execution time, microseconds |
| 49 | Bits: 1 accel, 2 gyro, 4 left, 8 right updated; 16 gravity calibrated; 32 numerically healthy |

Flags describe the most recent estimator cycle, not an absolute accuracy guarantee.
Latest sensor values can be older than the log row; update flags distinguish
fresh use. Logging remains 100 Hz, with a 400 kB buffer sized from `sizeof(LogData)`;
added fields reduce capture capacity to 17.54 seconds with the added trajectory IMU fields. Check the printed
actual capacity before longer experiments. Existing legacy pose columns remain
separate from the new estimates. Heading and travel origins are arbitrary;
X/Y is derived without a separate position covariance.

Tune measurement variances using stationary and steady-motion records; tune
linear/angular jerk noise against transients and external distance/angle
measurements. Bias observability depends on excitation and trustworthy encoders.
The estimate does not reliably distinguish sustained slip from tracking error.
Measure column 48 on hardware and ensure estimator plus sensor/control work
fits the 1 ms controller schedule. A successful build cannot establish timing.

The trajectory capture additionally appends columns 50–55 with all IMU axes;
see README_trajectory.md.

## Numerical verification

From the repository root after PlatformIO installs dependencies:

```sh
c++ -std=c++14 -O2 -Itests/motion_host -Isrc/src -Isrc/.pio/libdeps/teensy41/BasicLinearAlgebra tests/motion_host/check.cpp -o /tmp/motion-check
/tmp/motion-check > /tmp/motion-check.csv
```

The C++ harness checks forward/reverse turning, biases, covariance positive
definiteness and symmetry, timestamp wrap, variable timing, dropout prediction,
invalid observations, long-gap detection, and reset. Compare it with MATLAB:

```matlab
addpath('week06'); verify_motion_estimator('/tmp/motion-check.csv');
```

On hardware, additionally test stationary drift, acceleration/deceleration,
in-place turns, IMU disconnection, and distance/heading against external truth.
