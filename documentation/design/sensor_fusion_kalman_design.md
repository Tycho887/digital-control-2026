# Sensor-Fused Kalman Filter Design

Differential-drive robot: linear velocity / angular velocity / distance /
heading estimation.

Hardware used by the filter:

- MPU-9250 accelerometer and gyro.
- Two 48-PPR quadrature wheel encoders on the motor shafts.
- Optional AS5047P/AS5147 absolute-angle sensors (currently dead; retained as
  an upgrade path).

The AK8963 magnetometer is broken/unresponsive and is **not** used. It remains
a raw diagnostic sensor only. The authoritative implementation reference is
`src/src/KF_specification.md`; the gain generator is
`kalman_filter_tuning/solve_dare.m`.

---

## 1. State Vector and Conventions

```text
x = [ v, omega, b_a ]
```

| Symbol | Meaning | Unit |
| --- | --- | --- |
| `v` | forward velocity | m/s |
| `omega` | yaw rate | rad/s |
| `b_a` | residual accelerometer bias | m/s² |

**Integrated quantities are not states.** Distance `s`, heading `theta`, and
pose `x`, `y` are dead-reckoned outside the filter. The DARE therefore has no
unobservable integrator states and no magnetometer bias state.

The gyro bias is removed by the IMU driver's startup offset calibration; it is
not a filter state. With encoder-only wheel omega the DARE cannot identify a
separate `b_g` state, so omitting it avoids a nearly unobservable mode.

Conventions: `v > 0` forward, `omega > 0` CCW; left/right wheel linear
velocities `v_L`, `v_R`; wheel radius `r`; wheelbase `d`.

---

## 2. Process Model

The accelerometer enters as a control input `u = a_x` (body-frame forward
acceleration, bias-corrected by the filter):

```text
x_k+1 = F x_k + B u_k
```

```text
F = [ 1   0  -dt ]
    [ 0   1    0 ]
    [ 0   0    1 ]

B = [ dt ]
    [  0 ]
    [  0 ]
```

`a_x` is the raw forward accelerometer reading minus the startup average; the
filter subtracts `b_a`. `omega` is not propagated by the model; the gyro and
wheel-omega updates correct it.

Process noise:

```text
Q = B*(sigma_accel^2 + q_v^2)*B' + diag(0, (q_om*dt)^2, q_ba^2*dt)
```

- `sigma_accel` is the measured per-sample accelerometer noise.
- `q_v` is an unmodelled velocity disturbance (slip, bumps, drag), not sensor
  noise.
- `q_om` is the unmodelled angular acceleration; use the turn-limiter value.
- `q_ba` is the accelerometer bias random walk, from Allan variance when
  available.

The firmware solves the blocks at their actual rates: the `[v, b_a]` block is
lifted to 10 ms for the wheel-v update; the `omega` block uses a 1 ms DARE for
the gyro and a 10 ms DARE for the wheel-omega update.

---

## 3. Measurement Model

Measurements are stacked as raw scalar updates, not pre-combined into a
correlated vector:

```text
z = [ v_wheel ; omega_wheel ; omega_gyro ]
```

```text
H = [ 1  0  0 ]   wheel v
    [ 0  1  0 ]   wheel omega
    [ 0  1  0 ]   gyro omega
```

The wheel rows come from the encoder-derived left/right wheel velocities:

```text
v_wheel     = 0.5*(v_L + v_R)
omega_wheel = (v_R - v_L)/d
```

The gyro reading is already offset-corrected by the driver. No magnetometer row
exists.

Measurement noise:

```text
R = diag(sigma_wheel_v^2, sigma_wheel_omega^2, sigma_gyro^2)
```

| Entry | Value | Source |
| --- | --- | --- |
| `sigma_gyro` | 0.17 deg/s | 14 Nov `cal upside` phases 11/12 |
| `sigma_wheel_v` | 0.0145 m/s | 10 ms encoder difference |
| `sigma_wheel_omega` | 0.18 rad/s | 10 ms encoder difference |

The wheel `v` and wheel `omega` estimates share the same encoder counts, so
their noise is correlated. The firmware applies two scalar updates for code
size; the fixed gains absorb the small over-trust this creates.

---

## 4. Stationary Gains

The firmware propagates no covariance. The gain bank is the steady-state DARE
solution per block:

```text
[P, ~, ~] = dare(F', H', Q, R);
K = P*H'/(H*P*H' + R);
```

```text
K_WHEEL_V     = [ 0.197604, -0.018533 ] over [v, b_a]
K_GYRO        = 0.209730                over omega
K_WHEEL_OMEGA = 0.012222                over omega
```

The wheel-v first gain preserves roughly the previous 50 ms velocity step
response. The bias gain is slower because the DARE trusts the accelerometer
for short-term velocity and the wheel only for long-term correction. Tune the
`q_v` disturbance if a faster response is required.

`tests/kalman_host/verify_gains.py` checks the 100-step multi-rate error
transition and compares the header gains with a NumPy replica of the DARE.

---

## 5. Encoder Noise Model

Encoder velocity per window:

```text
v_wheel = DeltaN * (2*pi/48) * r / (gear * dt_window)
```

The quadrature encoder fallback is the active wheel source on the current
hardware. The 10 ms window and the starting speed set the quantization/timing
noise. The measured in-motion
1-sigma values in the 14 Nov captures are 0.013-0.0145 m/s for wheel `v` and
0.18 rad/s for wheel omega.

The optional AS5X47 path differences its absolute angle over the same 10 ms
window. Its datasheet angle noise is 0.068° RMS; when the sensor is repaired,
use the AS5X47 noise figure and re-solve the DARE.

Encoder timing jitter, wheel imperfection, and slip dominate the quantization
floor. Slip is environment-dependent; inflate `sigma_wheel_v` / `q_v` during
hard acceleration or turning rather than pretending the encoder is ideal.

---

## 6. Tuning Procedure

1. **Stationary IMU log** (at least 10 min): Allan variance gives the gyro
   bias random walk and accelerometer bias random walk. Replace the `q_ba`
   placeholder and confirm `sigma_gyro` / `sigma_accel`.
2. **Encoder noise captures**: compare encoder-derived wheel `v` and omega
   against the gyro or AS5X47 over several speeds; update
   `sigma_wheel_v` / `sigma_wheel_omega`.
3. **Power-on stationary calibration**: the driver removes the gyro offset;
   the firmware averages the accelerometer for 2 s and keeps the residual as
   `b_a`.
4. **Full-vehicle test**: drive straight and rotate in place; compare `v` and
   omega against the encoder and gyro. Adjust `q_v` / `q_om` if the response
   is too slow or too noisy, then regenerate the DARE gains.

---

## 7. External Dead Reckoning

After each filter tick:

```text
ds     = 0.5*(v_prev + v)*dt
dtheta = 0.5*(omega_prev + omega)*dt
mid    = wrap(theta + 0.5*dtheta)
x     += ds*cos(mid)
y     += ds*sin(mid)
s     += ds
theta  = wrap(theta + dtheta)
```

The log records the acceleration input, velocity, and yaw rate, so distance
can also be reconstructed offline with the trapezoidal rule. Heading is
relative to the run origin and has no absolute correction while the
magnetometer is unavailable.

---

## 8. Excluded Sensors

**Magnetometer.** The AK8963 reports zero valid headings in both 14 Nov
captures. It is not fused: no state, no gain, no update. The driver still
probes it and logs raw values/ST1/ST2 for diagnostics; `cal mag` and
`cal retrymag` remain available if the hardware is repaired. Re-enabling it
would require a new DARE row and a new absolute-heading state, not a silent
fallback.

**Gyro bias state.** Not estimated. The startup offset removes the bulk; the
encoder wheel-omega noise makes an online `b_g` state unobservable in the
current DARE. A future AS5X47 repair would make a bias state viable.

---

## Appendix: Minimal Parameters

```text
dt        = 1 ms (filter), 10 ms wheel window
sigma_gyro        = 0.17 deg/s
sigma_accel       = 0.12 m/s^2
sigma_wheel_v     = 0.0145 m/s
sigma_wheel_omega = 0.18 rad/s
q_v               = 1.0 m/s^2
q_om              = 0.7 rad/s^2
q_ba              = 3e-3 m/s^2/sqrt(s)
x0                = [0, 0, 0]
```
