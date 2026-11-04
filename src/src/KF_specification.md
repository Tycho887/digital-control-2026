# Kalman Filter Specification — MPU-9250 + 2× AS5047P/AS5147

Seven-state linear Kalman filter for a differential-drive robot (`kalman.h` /
`kalman.cpp`). Runs at 1 kHz. All quantities are SI: metres, seconds, radians.
The filter fuses the MPU-9250 accelerometer (control input), gyro (yaw rate),
magnetometer (heading), and two absolute magnetic angle sensors on the motor
shafts (instantaneous linear and angular velocity). There are **no quadrature
encoder measurements** in the filter.

---

## 1. Sensors used

| Sensor | Part | Signal used | Units | Rate | Role in the filter |
|--------|------|-------------|-------|------|--------------------|
| Gyroscope (yaw) | **MPU-9250** | `ω + b_g` | rad/s | 1 kHz | Turn-rate measurement; makes `ω` and `b_g` observable |
| Accelerometer (forward axis) | **MPU-9250** | `a + b_a` | m/s² | 1 kHz | Control input `u` for velocity propagation, not a measurement |
| Magnetometer (3-axis, AK8963 die) | **MPU-9250** | `θ + b_m` (mod 2π) | rad | 100 Hz (AK8963 continuous mode), innovation-gated | Absolute heading anchor; makes `b_m` observable |
| Absolute angle, left motor shaft | **AS5047P** or **AS5147** | angle φ_L over SPI (`AS5X47::readAngle()`) | rad | 100 Hz (10 ms window) | Instantaneous wheel speed and turn rate |
| Absolute angle, right motor shaft | **AS5047P** or **AS5147** | angle φ_R over SPI (`AS5X47::readAngle()`) | rad | 100 Hz (10 ms window) | Instantaneous wheel speed and turn rate |

The two AS5X47 sensors replace the wheel encoders as the velocity reference.
They supply `v` and `ω` only; they do not feed absolute distance or heading.

---

## 2. Explicitly out of scope / not used

- **Stock 48-edge quadrature motor encoders (`uencoder`).** No `encoder.*`
  quantity enters `predict()` or any `update*()` call. The quadrature encoders
  remain wired and in use **only** by the wheel-speed PI controller.
- **Wheel- or encoder-derived absolute distance `s`.** `s` is integrated from
  the filter's own `v`; there is no `[1,0,0,0,0,0,0]` measurement row.
- **Wheel- or encoder-derived absolute heading `θ`.** `θ` is integrated from
  `ω`; heading is anchored by the gyro (short term) and the magnetometer
  (long term), not by differencing wheel angles. This keeps wheel slip out of
  the pose estimate.
- **AS5X47 velocity register (`readVel()`).** Not used; see Section 5.

Because there is no position measurement, `s` is dead-reckoned throughout and
`θ` is dead-reckoned between magnetometer updates. Drift in `s` is bounded
only by the accuracy of `v`, and drift in `θ` only by the gyro bias estimate
and the magnetometer.

---

## 3. State vector

```
x = [ s, θ, v, ω, b_a, b_g, b_m ]ᵀ
```

| # | Symbol | Meaning | Unit | Initial 1-σ (`P0`) | Initial variance |
|---|--------|---------|------|--------------------|------------------|
| 0 | `s` | travelled distance, ∫v dt | m | `1e-2` | `1e-4` m² |
| 1 | `θ` | heading / total turn angle, ∫ω dt | rad | `1e-1` | `1e-2` rad² |
| 2 | `v` | forward velocity | m/s | `1e-1` | `1e-2` (m/s)² |
| 3 | `ω` | turn rate | rad/s | `1e-1` | `1e-2` (rad/s)² |
| 4 | `b_a` | accelerometer bias, forward axis | m/s² | `2e-1` | `4e-2` (m/s²)² |
| 5 | `b_g` | gyro bias, yaw axis | rad/s | `1e-2` | `1e-4` (rad/s)² |
| 6 | `b_m` | magnetometer heading offset | rad | `1e-1` | `1e-2` rad² |

`P0 diag = [1e-4, 1e-2, 1e-2, 1e-2, 4e-2, 1e-4, 1e-2]`. These are tuning
values, not datasheet numbers.

---

## 4. Noise parameters

### 4.1 Measurement noise `R` (scalar per update, `R = σ²`)

| Parameter | Symbol in code | Formula | 1-σ value | `R` (variance) | Units of `R` | Source |
|-----------|----------------|---------|-----------|----------------|--------------|--------|
| Gyro | `R_gyro` | `(0.1·π/180)²` | `1.745e-3` rad/s | `3.046e-6` | (rad/s)² | MPU-9250 Table 1, total RMS noise 0.1 °/s at DLPFCFG=2 (92 Hz) |
| AS5X47 linear velocity | `R_wheel_v` | `(√2·r·σ_angle/(gear·T))²` | `5.51e-4` m/s | `3.03e-7` | (m/s)² | AS5047P/AS5147 RMS angle noise 0.068° (1σ, NOISESET=0), endpoint difference over T = 10 ms |
| AS5X47 turn rate | `R_wheel_omega` | `(√2·2·r·σ_angle/(b·gear·T))²` | `7.87e-3` rad/s | `6.19e-5` | (rad/s)² | same, plus wheelbase `b` |
| Magnetometer | `R_mag` | engineering estimate `(0.01)²` | `1.0e-2` rad | `1.0e-4` | rad² | no RMS figure in the MPU-9250 datasheet; quantisation floor in §4.4 |

Constants: wheel radius `r = 0.0315 m`, wheelbase `b = 0.14 m`, reduction
ratio `gear = 9.6` (motor shaft to wheel), difference window `T = 0.01 s`,
`σ_angle = 0.068° = 1.187e-3 rad`.

### 4.2 Accelerometer noise (enters `Q`, not `R`)

| Quantity | Formula | Value | Units | Source |
|----------|---------|-------|-------|--------|
| Noise density | — | `300e-6` | g/√Hz | MPU-9250 Table 2, low-noise mode |
| 1-σ in 94 Hz bandwidth | `300e-6 · 9.80665 · √94` | `0.0285` (≈ 2.9 mg) | m/s² | DLPF set to 94 Hz |

The accelerometer is a control input, so its noise is propagated into `Q`:

```
Q_ss = ¼ σ_a² dt⁴
Q_sv = Q_vs = ½ σ_a² dt³
Q_vv = σ_a² dt² + q_v · dt
```

### 4.3 Process-noise random walks (`Q`, per second) — tuning values

| Parameter | Symbol | Value | Units |
|-----------|--------|-------|-------|
| Forward-velocity random walk | `q_v` | `1e-2` | (m/s)²/s |
| Turn-rate random walk | `q_omega` | `1e-2` | (rad/s)²/s |
| Accelerometer bias random walk | `q_ba` | `1e-6` | (m/s²)²/s |
| Gyro bias random walk | `q_bg` | `1e-8` | (rad/s)²/s |
| Mag heading offset random walk | `q_bm` | `1e-10` | rad²/s |

Remaining diagonal entries: `Q_ωω = q_omega·dt`, `Q_ba,ba = q_ba·dt`,
`Q_bg,bg = q_bg·dt`, `Q_bm,bm = q_bm·dt`. These numbers are hand-tuned, not
datasheet-derived; an Allan-variance analysis is the proper source.

### 4.4 Where each number comes from

- **Gyro, `0.1 °/s`:** total RMS noise over the configured 92 Hz DLPF
  bandwidth. Rate noise density is `0.01 °/s/√Hz`; integrating that over the
  bandwidth gives roughly `0.1 °/s`, so the RMS figure already includes the
  bandwidth and must not be scaled by `√Δt` again. `R_gyro` is used as a
  per-sample variance at 1 kHz.
- **Accelerometer, `300 µg/√Hz`:** a density, so it must be multiplied by the
  square root of the bandwidth. At 94 Hz this is `0.0285 m/s²` RMS. The old
  `8 mg` value was not a datasheet figure for this bandwidth and overstated
  the true RMS noise by about 2.8×.
- **AS5X47, `0.068°`:** RMS output noise, 1σ, with `NOISESET=0`, 14-bit core
  resolution (1 LSB = 0.02197°). AS5147 also lists `0.082°` for
  `NOISESET=1`. Integral non-linearity (INL) is quoted separately:
  ±0.8° (25 °C, optimum magnet placement), ±1.0° (optimum placement over
  temperature), ±1.2° (magnet displaced over temperature). INL is
  **systematic**, not white noise: it belongs in the angle scale/offset
  uncertainty, not in `R`. Squared, ±0.8° would be `1.95e-4 rad²`, about
  640× `R_wheel_v`, so folding it into `R` would make the wheel updates
  useless. The endpoint difference of two angle readings has `√2` times the
  per-sample RMS, which is why the formulas above carry the `√2`.
- **Magnetometer, `0.01 rad`:** no RMS noise figure exists in the MPU-9250
  datasheet. Only sensitivity is given: `0.6 µT/LSB` (14-bit) and
  `0.15 µT/LSB` (16-bit AK8963 mode, which the firmware uses). At a 17 µT
  horizontal field, one LSB is `0.15/17 = 8.8 mrad`; the RMS of uniform
  quantisation is `LSB/√12 ≈ 2.5 mrad`. The retained `0.01 rad` is about 4×
  this floor, left conservative because hard-iron distortion and the magnetic
  environment around the motors dominate quantisation. Treat it as an
  engineering estimate, not a datasheet value.

---

## 5. AS5X47 velocity derivation

Two angle readings `φ(t−T)` and `φ(t)` are differenced over `T = 10 ms`
(100 Hz). With the sensor on the motor shaft (reduction ratio `gear = 9.6`,
i.e. the motor shaft turns 9.6 times per wheel turn):

```
v_wheel,i   = (r / gear) · Δφ_i / T                     (m/s)
v           = ½ (v_L + v_R)                             (m/s)
ω           = (v_R − v_L) / b                           (rad/s)
```

Only the angle difference is used, so the sensor's absolute zero position and
its slow drift cancel. `v_L`, `v_R` take the sign convention of the motor
direction; the left sensor is inverted relative to the right, matching the PI
controller's `vLeft = −motorVelocity[0]·r/gear` convention.

Each angle reading has RMS noise `σ_angle`; the endpoint difference therefore
has `√2·σ_angle`. Substituting that into the velocity equations gives the `R`
values in §4.1:

```
σ_v  = √2·r·σ_angle / (gear·T)      = 1.414·0.0315·1.187e-3 / (9.6·0.01) = 5.51e-4 m/s
σ_ω  = √2·2·r·σ_angle / (b·gear·T)  = 1.414·2·0.0315·1.187e-3 / (0.14·9.6·0.01) = 7.87e-3 rad/s
```

**Why not `readVel()`.** The AS5X47 velocity register (`ANGLE_VEL`, 0x3FFC)
has an LSB of `24.141 °/s`. At the nominal 0.02 m/s drive command, the wheel turns at
`0.02/0.0315 = 0.635 rad/s = 36.4 °/s`, i.e. only about 1.5 counts at the wheel
(≈ 14 counts at the motor shaft). That is far too coarse to be a measurement.
Differencing 14-bit angles over 10 ms resolves velocities about an order of
magnitude finer: one angle LSB per 10 ms is `3.83e-4/0.01 = 0.0383 rad/s` at
the sensor shaft.

**Note on correlation.** Both wheel updates derive from the same two angles,
so their noises are correlated (the `v` and `ω` errors both come from
`Δφ_L ± Δφ_R`). Applying them as two independent scalar updates slightly
over-trusts the wheel measurement; a joint 2×2 update would be exact. The
scalar form is kept for code size and matches the rest of the filter.

---

## 6. Measurement models

Each measurement is scalar: `z = H·x + n`, `n ~ N(0, R)`, `H` is a 1×7 row over
`[s, θ, v, ω, b_a, b_g, b_m]`. Angle measurements are wrapped into `(−π, π]`
before the update.

| Sensor | `H` row | `z` | `R` |
|--------|---------|-----|-----|
| Gyro | `[0, 0, 0, 1, 0, 1, 0]` | `ω + b_g` | `3.046e-6` (rad/s)² |
| Magnetometer | `[0, 1, 0, 0, 0, 0, 1]` | `θ + b_m` (wrapped) | `1.0e-4` rad² |
| AS5X47 linear velocity | `[0, 0, 1, 0, 0, 0, 0]` | `v` | `3.03e-7` (m/s)² |
| AS5X47 turn rate | `[0, 0, 0, 1, 0, 0, 0]` | `ω` | `6.19e-5` (rad/s)² |
| Accelerometer | — | — | control input `u = a_meas`, folded into `Q` |

Update order per cycle: `predict(a, dt)` → `updateGyro` (whenever a fresh
sample exists) → (every 10 ms) `updateWheelV`, `updateWheelOmega` → `updateMag`
(whenever a fresh heading exists, about 100 Hz). The magnetometer update is
innovation-gated: `y² > 9·S` rejects the sample (`magRejected`).

---

## 7. Observability summary

| State | Made observable by | Comment |
|-------|--------------------|---------|
| `s` | nothing directly | Integrated from `v`; dead-reckoned, drifts slowly |
| `θ` | gyro + magnetometer | Integrated from `ω`; anchored by `b_m`-corrected mag heading |
| `v` | AS5X47 linear velocity | Direct measurement every 10 ms |
| `ω` | gyro, plus AS5X47 turn rate | Two independent references at different rates |
| `b_a` | mismatch between integrated accel and AS5X47 `v` | Weak: only observable through the velocity residual, so it converges slowly and is sensitive to `R_wheel_v` |
| `b_g` | gyro vs AS5X47 `ω` (fast), mag heading (slow) | Well observed at 100 Hz, since the wheel turn rate is independent of the gyro |
| `b_m` | mag heading vs gyro-integrated `θ` | Observable whenever the robot turns in a steady field |

Consequences of dropping wheel position updates:

- `s` and `θ` have no absolute reference in the measurement model; both are
  pure integration. `θ` is still bounded by the magnetometer, `s` is not
  bounded by anything and will drift with `v` bias.
- `b_a` is now the weakest state. With no `s`/`θ` rows, the only evidence about
  the accelerometer bias is the disagreement between the double-integrated
  acceleration and the wheel-speed measurement. Keeping `P0[4][4] = 4e-2`
  and `q_ba` small avoids letting it wander.
- If a sensor is dropped, disable its `update…()` call **and** pin the
  affected `P[i][i]` to a realistic value; otherwise the state can go
  unobservable.

---

## 8. Datasheet cross-reference

| Number | Value | Source |
|--------|-------|--------|
| Gyro total RMS noise | `0.1 °/s` at DLPFCFG=2 (92 Hz) | MPU-9250 datasheet, Table 1 (Gyroscope Specifications) |
| Gyro rate noise density | `0.01 °/s/√Hz` | MPU-9250 datasheet, Table 1 |
| Accel noise power spectral density | `300 µg/√Hz`, low-noise mode | MPU-9250 datasheet, Table 2 (Accelerometer Specifications) |
| Mag sensitivity | `0.6 µT/LSB` (14-bit); `0.15 µT/LSB` (16-bit AK8963) | MPU-9250 datasheet, §3.3 (Magnetometer Specifications) |
| AS5X47 core resolution | 14 bit | AS5047P / AS5147 datasheet, System Characteristics |
| AS5X47 RMS output noise | `0.068°` (1σ, NOISESET=0); AS5147: `0.082°` (NOISESET=1) | AS5047P / AS5147 System Characteristics |
| AS5X47 INL | ±0.8° (OPT), ±1.0° (OPT+TEMP), ±1.2° (DIS+TEMP) | AS5047P / AS5147 System Characteristics |
| AS5X47 velocity LSB | `24.141 °/s` | AS5047P / AS5147 datasheet, velocity register |

OPT = optimum magnet placement at 25 °C; DIS = magnet displaced; TEMP = over
the operating temperature range.

---

## 9. Implementation notes

- **DLPF.** `uimu2.cpp` writes `CONFIG` (0x1A) and `ACCEL_CONFIG2` (0x1D) with
  `DLPF_CFG = 2` after `beginAccel`/`beginGyro`: 92 Hz gyro, 94 Hz accel.
  The `MPU9250_asukiaaa` library does not set the DLPF itself, and the
  datasheet noise figures in §4 only apply at this bandwidth. Changing the
  DLPF changes `R_gyro` and `σ_a`.
- **SPI wiring (assumption).** Left CS = pin 24, right CS = pin 25, hardware
  SPI on 11/12/13 (MISO/MOSI/SCK), `SPI_MODE1` at 1 MHz. The pins are
  configurable constants and must be confirmed against the harness; they are
  not yet verified on hardware.
- **Gear / mounting (assumption).** The sensors sit on the motor shafts, where
  the stock encoders were, so wheel angle = sensor angle / 9.6. If a sensor is
  mounted directly on a wheel, set `gear = 1` in one place; `R_wheel_v` and
  `R_wheel_omega` then follow from the same formulas.
- **Constants.** `gear` (9.6), `wheelRadius` (0.0315 m) and `wheelBase`
  (0.14 m) also exist in `basebot_6.ino`; the filter mirrors them as
  `KF_AS5X47_GEAR`, `KF_WHEEL_RADIUS` and `KF_WHEEL_BASE` in `kalman.h`, so
  keep the two sets in sync. The filter computes the two wheel `R` values from
  `KF_AS5X47_SIGMA_ANGLE` and the actual update window, so a different `T`
  rescales them automatically (`R ∝ 1/T²`).
- **Encoder path.** `uencoder` still runs and still drives the wheel-speed PI
  loop; it must not be re-added to the estimator.
- **Precision / Python parity.** C++ uses `float` (24-bit mantissa, ~7 decimal
  digits); the Python reference uses `float64`. Bit-exact equality is not
  expected. Compare states and `P` diagonals with a tolerance (`rtol ≈ 1e-4`,
  `rtol ≈ 1e-3` for the nearly unobservable `θ`/`b_m` pair, plus an absolute
  floor for near-zero states such as `s` at the start), and compare
  `magRejected` flags exactly. Larger divergence points to a real algorithm
  difference, not rounding.
- **Covariance hygiene.** `P` is symmetrised after every predict/update, and
  the state is checked for finiteness (`healthy`); a non-finite state stops
  updates rather than propagating NaNs.
