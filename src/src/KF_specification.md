# Stationary Estimator Specification — MPU-9250 + 2× AS5047P/AS5147

Constant-gain, decoupled estimator for the planar differential-drive robot
(`kalman.h` / `kalman.cpp`). The filter runs at 1 kHz. All quantities are SI:
metres, seconds, radians. A reusable incremental PID (`pid.h`) closes the
outer forward-velocity and yaw-rate loops around the estimator.

The older 7-state covariance-propagating filter and `week08/DARE.m` are
deprecated. They remain in git history and in `src/references/main.py` as
historical references, but they are not the firmware baseline.

---

## 1. Why the filter is stationary

The filter state contains only rates and biases:

```
x = [ v, b_a, theta, omega, b_g ]
```

Position `s`, `x`, and `y` are deliberately **not** states. The firmware
dead-reckons them from `v` and `theta` after every predict, outside the filter.
With no position state and no covariance propagation, the filter is
time-invariant: every update is a fixed linear correction and the gains are
constant.

| Symbol | Meaning | Unit |
|--------|---------|------|
| `v` | forward velocity | m/s |
| `b_a` | accelerometer bias (residual after startup average) | m/s² |
| `theta` | heading | rad |
| `omega` | yaw rate | rad/s |
| `b_g` | gyro bias | rad/s |

---

## 2. Prediction (1 kHz)

```
v     += (a_raw - b_a) * dt
theta  = wrapAngle(theta + omega * dt)
```

`a_raw` is the forward accelerometer reading in m/s². The firmware subtracts a
2 s stationary startup average before passing it in, so `b_a` tracks the
residual offset. `dt` is the measured tick interval and is rejected outside
`(0, 20] ms`.

`omega` and `b_g` are not propagated by the model; they are corrected by the
gyro and wheel-yaw-rate measurements below.

---

## 3. Updates

Each measurement is a scalar correction `x += K * (z - H*x)`.

| Update | Rate | `H` row | Gain vector | State block |
|--------|------|---------|-------------|-------------|
| AS5X47 wheel `v` | 100 Hz (10 ms window) | `[1, 0]` | `K_WHEEL_V` | `[v, b_a]` |
| MPU gyro `omega + b_g` | 1 kHz | `[0, 1, 1]` | `K_GYRO` | `[theta, omega, b_g]` |
| AS5X47 wheel `omega` | 100 Hz (10 ms window) | `[0, 1, 0]` | `K_WHEEL_OMEGA` | `[theta, omega, b_g]` |
| Magnetometer heading | async, gated | `[1, 0, 0]` | `K_MAG` | `[theta, omega, b_g]` |

Current gain bank in `kalman.h`:

```cpp
K_WHEEL_V     = { 0.18f, -0.35f }
K_GYRO        = { 0.002f, 0.85f, 0.0005f }
K_WHEEL_OMEGA = { 0.01f, 0.12f, -0.04f }
K_MAG         = { 0.05f, 0.0f, -0.002f }
MAG_GATE_RAD  = 0.25f
```

The magnetometer update rejects `|wrap(magHeading - theta)| > 0.25 rad`
(about 14°). The first valid heading initialises `theta` directly; subsequent
samples use the gated update.

The wheel `v` and wheel `omega` measurements both come from the same two
AS5X47 angles, so their noise is correlated. They are applied as two scalar
updates for code size; the fixed gains absorb the small over-trust this
creates.

---

## 4. Gain stability at the real update schedule

The gains are not derived from a single-rate DARE. The firmware schedule is
mixed: predict and gyro at 1 kHz, wheel v/omega every 10 ms, magnetometer
every 100 ms. `tests/kalman_host/verify_gains.py` builds the 100-step error
transition for that exact schedule. Current result:

```
individual update |eig| (unobservable modes remain at 1.0):
  gyro         0.1495  1.0  1.0  1.0  1.0
  wheel v      0.8216  0.9980 1.0  1.0  1.0
  wheel omega  0.8800  1.0  1.0  1.0  1.0
  mag          0.9500  1.0  1.0  1.0  1.0

100-step cycle max |eig| = 0.949992
```

The unit eigenvalues are expected for modes that a single update cannot
observe (for example, `theta` from a pure rate measurement). The combined
cycle is stable. The deterministic simulation in the same script converges to
`v = 0.200000`, `b_a = 0.049999`, `omega = 0.200000`, and `b_g = 0.010000`.

When the stationary DARE is regenerated, replace the gain bank in
`kalman.h`. If the DARE is solved at a different measurement interval, lift
the model to the actual 10 ms wheel interval before copying the gain; do not
apply a 1 ms gain at 100 Hz.

---

## 5. External dead reckoning

After each predict, `estimatorUpdate()` integrates:

```
ds    = 0.5 * (v_prev + v) * dt
dtheta = wrapAngle(theta - theta_prev)
mid   = wrapAngle(theta_prev + 0.5 * dtheta)
x    += ds * cos(mid)
y    += ds * sin(mid)
s    += ds
poseHeading = wrapAngle(theta - theta_at_start)
```

`start()` captures `theta_at_start`, resets `s = x = y = 0`, and seeds
`v_prev` / `theta_prev`. The estimator's learned biases persist across runs.

---

## 6. Outer control loops

`src/src/basebot_6.ino` runs a cascade every 1 ms:

```
vError     = vRef - estimator.v
omegaError = omegaRef - estimator.omega

vCmd     = vRef     + velocityPid.ComputeCCDE(vError)
omegaCmd = omegaRef + yawRatePid.ComputeCCDE(omegaError)

left,right = polar2Cartesian(vCmd, omegaCmd) * (gear / wheelRadius)
```

The inner wheel-speed PI on the quadrature encoders receives motor-shaft
rad/s. The outer PID outputs are clamped to ±0.30 m/s and ±2.00 rad/s. The
incremental form clamps before storing `u[k]`, so it does not wind up.

The outer coefficients are placeholders in `basebot_6.ino`:

```cpp
vel_pid_coeffs = { 0.50f, -0.49f, 0.0f, -1.0f, 0.0f };
yaw_pid_coeffs = { 0.40f, -0.39f, 0.0f, -1.0f, 0.0f };
```

They are defined in one block specifically so they can be replaced after
tuning without touching the control path.

---

## 7. Log format 2

The standard serial log no longer contains a covariance. The estimator
columns are:

| Columns | Content |
|---------|---------|
| 27-28 | estimator `v`, `omega` |
| 29 | magnetometer heading |
| 30-31 | wheel controller setpoint (left, right) |
| 32 | yaw-rate reference |
| 33-37 | estimator state `[v, omega, theta, b_a, b_g]` |
| 38 | forward-velocity error |
| 39 | yaw-rate error |
| 40 | outer-loop forward-velocity command |
| 41 | outer-loop yaw-rate command |

The calibration capture keeps four innovation slots (gyro, wheel v, wheel
omega, mag). Its innovation-variance columns are zero because the stationary
filter does not propagate `S`; the analyzer skips NIS for these captures.

---

## 8. Verification

```sh
# Firmware
cd src && pio run -e teensy41

# Estimator and PID host checks
c++ -std=c++14 -O2 -Isrc/src tests/kalman_host/check.cpp src/src/kalman.cpp \
    -o /tmp/kalman-check && /tmp/kalman-check
c++ -std=c++14 -O2 -Isrc/src tests/motion_host/controller_check.cpp \
    -o /tmp/controller-check && /tmp/controller-check

# Gain stability and C++/NumPy parity
tests/kalman_host/.venv/bin/python tests/kalman_host/verify_gains.py
tests/kalman_host/.venv/bin/python tests/kalman_host/test_parity.py
```

---

## 9. Implementation notes

- **Finite guards.** `predict` and every update reject non-finite inputs and
  leave the state unchanged. `healthy` goes false if any state becomes
  non-finite.
- **Angle handling.** `theta` is wrapped to `(-π, π]` after every integrate or
  correction. Relative pose heading is wrapped at the start origin.
- **Update order.** `predict` → `updateGyro` → `updateWheelV` →
  `updateWheelOmega` → `updateMag`, matching the firmware tick.
- **AS5X47 velocity.** The absolute angles are differenced over a 10 ms
  window; `readVel()` is not used because its 24.141 °/s LSB is too coarse at
  low speed.
- **Wheel geometry.** `gear = 9.6`, `wheelRadius = 0.0315 m`,
  `wheelBase = 0.14 m` are mirrored between `basebot_6.ino` and the AS5X47
  conversion; keep them in sync.
- **Deprecated DARE.** `week08/DARE.m` and `week08/static_kf.mat` describe the
  old 4-state covariance filter. They are marked deprecated and are not the
  source of the current gains.
