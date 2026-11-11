# Sensor-Fused Kalman Filter Design

Differential-drive robot: linear velocity / angular velocity / distance / heading estimation
Hardware: MPU-9250 (IMU with AK8963 magnetometer) + two AS5047P/AS5147 absolute magnetic angle sensors on the motor shafts (SPI). The stock 48-edge quadrature encoders still drive the wheel-speed PI loop, but the Kalman filter does not use them.

---

## Implementation status

The firmware implements the 7-state variant in `src/src/kalman.h` and
`src/src/kalman.cpp`, running at 1 kHz. It differs from the original design
below in these ways:

- **Sensor set:** MPU-9250 accelerometer/gyro/magnetometer and two
  AS5047P/AS5147 absolute angles read over SPI. The stock quadrature encoders
  are not filter measurements.
- **Extra magnetometer bias state:** the state is
  `[s, θ, v, ω, b_a, b_g, b_m]`; `b_m` absorbs the magnetometer heading offset.
- **Scalar updates:** each measurement is a separate scalar update with a 1×7
  `H` row, so there is no 3×3 innovation inverse.
- **No wheel position measurements:** the AS5X47 angles are differenced over
  10 ms to give instantaneous `v` and `ω`; wheel-derived distance `s` and
  heading `θ` are not used. `s` and `θ` are integrated, and the magnetometer
  anchors heading.
- **Datasheet noise:** gyro `0.1 °/s` RMS at DLPF 92 Hz, accelerometer
  `300 µg/√Hz` over the 94 Hz DLPF band, and AS5X47 `0.068°` RMS angle noise.
  The DLPF is configured to 92/94 Hz in `uimu2.cpp`.
- **Accelerometer as input:** forward acceleration enters the predict step as
  the control input; its noise is folded into `Q`.
- **Magnetometer enabled by default:** the heading update is innovation-gated
  and runs unless disabled with `mag 0`.
- **Calibration capture:** `cal mag`, `cal noise` and `cal track` record raw
  sensors, controller signals and filter innovations into a compact buffer;
  `src/scripts/analyze_calibration.py` checks the magnetometer axis, sensor
  noise and PI tracking.

Sections 1–7 and the appendix below describe the original quadrature-encoder
design and are retained as background.

## Calibration Procedure and Acceptance Checks

The firmware assumptions under test are: IMU X is the vertical/down axis,
forward is `-acc[2]`, CCW yaw is `-gyro[0]`, and magnetometer heading is
`atan2(mag[1], -mag[2])`. `cal mag` (motors off, 200 Hz for 15 s) asks for a
360° hand rotation. The analyzer reports the accelerometer gravity axis, the
field component with the smallest rotation variation, and the slope/RMSE of
each candidate heading pair against gyro-integrated yaw. Acceptance:

- The gravity axis and the smallest-variation field axis agree (expected X).
- The firmware heading slope is within about ±0.25 of +1 with RMSE below about
  0.2 rad. A slope near -1 means the sign is reversed; a different best pair
  means the axis mapping is wrong.

`cal noise` (motors off, 1 kHz for 3 s) measures per-sample 1-sigma for gyro X
(assumed 0.1 °/s), accel Z (0.0285 m/s²), AS5X47 angle (0.068°), 10 ms wheel
velocity (5.51e-4 m/s) and turn rate (7.87e-3 rad/s), and mag heading
(0.01 rad). A measured/assumed ratio between roughly 0.5 and 2 is treated as
plausible. The same capture reports mean normalized innovation squared
`y²/S` per update; NIS ≈ 1 is consistent, NIS ≫ 1 means `R` is too small or
`P` too confident. These are seeds for a retune, not an automatic R change.

`cal track` runs the week07-tuned PI on the quadrature encoders at 500 Hz.
It reports rise time, overshoot, settling time, steady-state/RMS error and
3 V saturation for the PI feedback, AS5X47 velocity and KF velocity. The
AS5X47/KF columns are an independent check on the encoder feedback; the
current 3 V limit is deliberately conservative, so saturation is reported
rather than hidden.

Magnetometer bring-up is checked with both the AK8963 WHO_AM_I register and a
delayed data read (ST1 DRDY, ST2 HOFL). `cal retrymag` repeats that probe. A
`NaN` mag heading in a standard log should be diagnosed this way before the
axis result is interpreted.

---

## 1. State Vector and Conventions

**State:**

```javascript
x = [s, θ, v, ω, b_g, b_a]ᵀ
     s   accumulated distance (m)
     θ   accumulated heading (rad)
     v   linear velocity (m/s)
     ω   angular velocity (rad/s)
     b_g gyro bias (rad/s)
     b_a accelerometer bias (m/s²)
```

Including b_g and b_a as states (instead of crudely inflating R) lets the filter estimate the biases online and avoids mis-weighting a constant offset as white noise.

**Conventions:** v > 0 = forward, ω > 0 = CCW; left wheel ω_L, right wheel ω_R (rad/s); wheel radius r; wheelbase d.

---

## 2. Process Model (Prediction Step)

The accelerometer enters as a **control input** u = a_x (body-frame longitudinal acceleration, bias-corrected). State propagation is linear:

```javascript
x⁻_k = F x_{k-1} + B u_k
```

```javascript
F = ⎡ 1  0  Δt   0    0     0   ⎤
    ⎢ 0  1   0  Δt    0     0   ⎥
    ⎢ 0  0   1   0    0     0   ⎥
    ⎢ 0  0   0   1   −Δt    0   ⎥   ← gyro reading contains bias b_g; subtract
    ⎢ 0  0   0   0    1     0   ⎥   ← b_g modeled as random walk
    ⎣ 0  0   0   0    0     1   ⎦   ← b_a modeled as random walk

B = [ Δt²/2, 0, Δt, 0, 0, 0 ]ᵀ
```

Note: u_k is the raw accelerometer reading minus the current b_a estimate.

**Process noise covariance Q** (6×6, near-diagonal). Q has no datasheet values — gyro/accel bias random walk must come from an Allan-variance analysis of stationary IMU logs. If no Allan data is available yet, initialize from the table below and tune with measured data:

| Param | Initial suggestion | Meaning |
| --- | --- | --- |
| Q_s | 0 | distance carries no process noise |
| Q_θ | 0 | heading carries no process noise |
| Q_v | (0.05·Δt)² | longitudinal velocity random disturbance (bumps) |
| Q_ω | (0.01·Δt)² | angular velocity random disturbance |
| Q_bg | (1e-5)²·Δt | gyro bias random walk (Allan flat region) |
| Q_ba | (1e-3)²·Δt | accelerometer bias random walk |

> Stationary Allan-variance analysis: the white-noise region (slope −1/2) gives the measurement noise density; the flat floor gives bias instability. This is the only reliable source for Q_bg, Q_ba, and the IMU terms of R.

---

## 3. Measurement Model (Update Step)

**Measurement vector (raw sensors stacked, not pre-fused):**

```javascript
z = [ω_L, ω_R, ω_gyro]ᵀ
```

```javascript
H = ⎡ r/2   r/2    0    0   0  0 ⎤   ← v = r/2·(ω_R + ω_L)
    ⎢ r/d  −r/d    0    0   0  0 ⎥   ← ω = r/d·(ω_R − ω_L)
    ⎣  0     0     0    1   1  0 ⎦   ← gyro reading = ω + b_g
```

New states s, θ have zero columns in H — they are not measured directly, only propagated.

**Measurement noise matrix:**

```javascript
R = diag( σ²_ωL, σ²_ωR, σ²_gyro )
```

| Entry | Value | Source |
| --- | --- | --- |
| σ²_gyro | 1.5×10⁻⁶ (rad/s)² @100Hz | datasheet noise density 0.01°/s/√Hz × √(f_s/2); bandwidth depends on actual DLPF setting |
| σ²_ωL / σ²_ωR | from calibration, varies with speed/window length | see Section 5 |

**Do NOT pre-combine encoders into v_enc / ω_enc and put both in z**: both derive from the same two noisy quantities, giving R a non-zero off-diagonal covariance (cov = r²/2d · (σ²_ωL − σ²_ωR)). Ignoring it makes the filter over-trust directions where the two "independent" measurements agree by construction. Stacking raw measurements sidesteps this entirely.

---

## 4. Discrete Kalman Equations (Embedded Implementation)

Prediction step (runs every IMU sample, e.g. 100 Hz):

```javascript
1)  x⁻ = F x + B (a_raw − b_a)
2)  P⁻ = F P Fᵀ + Q
```

Update step (runs whenever a new measurement arrives; gyro every cycle, encoders per window):

```javascript
3)  y  = z − H x⁻                (innovation)
4)  S  = H P⁻ Hᵀ + R             (innovation covariance, 3×3)
5)  K  = P⁻ Hᵀ S⁻¹               (Kalman gain, 6×3)
6)  x⁺ = x⁻ + K y
7)  P⁺ = (I − K H) P⁻            (or Joseph form, below)
```

**Embedded implementation notes:**

- **S⁻¹ is a 3×3 inverse**: write out the adjugate formula or do row Gaussian elimination; no numeric library needed.
- **Joseph-form covariance update** (numerically more stable, recommended):
`P⁺ = (I − K H) P⁻ (I − K H)ᵀ + K R Kᵀ`
- **Matrix sparsity**: F has ~10 nonzeros, H has 6. Expand element-wise and skip zero products — the full 6×6 P update needs only ~200 multiplications.
- **Two-rate structure**: prediction runs every IMU cycle; encoder measurements accumulate over a fixed window (e.g. 20–50 ms) and inject as one update, while the gyro can inject every cycle. Compute the encoder entries of R from the current window length and speed (Section 5).
- **Initialization:**

```javascript
  x₀ = [0, 0, 0, 0, b_g_cal, b_a_cal]ᵀ      (from 1–2 s of stationary averaging at power-on)
  P₀ = diag(0, 0, 1, 1, (5°/s→0.087)², (80 mg→0.784)²)
       ↑ bias initial uncertainty = datasheet zero-offset tolerance
```

---

## 5. Encoder Noise Model (R Encoder Entries)

Encoder velocity computed per window: `ω_wheel = ΔN·(2π/48) / Δt_window`

Quantization floor (timer resolution σ_t, e.g. 1 µs):

```javascript
σ²_ω ≈ ω² · [ (σ_t/Δt)² + (Δθ_q/(ω·Δt))² ] / 12
```

with Δθ_q = 2π/48 = 0.131 rad. **Measure in practice**: use the gyro as reference (pure-rotation test) and compare against the encoder-derived wheel-speed difference; the residual variance is σ²_ωL/σ²_ωR. The quantization floor is only a starting value: wheel-radius error is a systematic error (belongs in the scale uncertainty of P, not in R), and slip is environment-dependent (inflate R during hard acceleration/turning).

**Key datasheet values (MPU-9250, ±4 g / ±1000°/s configuration):**

| Parameter | Value |
| --- | --- |
| Gyro noise density | 0.01 °/s/√Hz |
| Gyro initial zero-rate offset tolerance | ±5 °/s (→ P₀,bg) |
| Gyro zero-rate offset drift over temperature | up to ±30 °/s (−40 to +85 °C, no numeric table) |
| Accelerometer noise density | 300 µg/√Hz |
| Accelerometer zero-g offset tolerance | ±60 mg (X,Y) / ±80 mg (Z) (→ P₀,ba) |
| Gyro/accel sensitivity error | ±3% |

The firmware never verifies the chip ID, and the part is discontinued with many counterfeits on the market — **read WHO_AM_I (expect 0x71) and compare stationary measured noise against the table above**; if measured noise is 3–5× higher, you have a clone — tune R and Q from measurements instead.

---

## 6. Optional: Magnetometer Heading Anchor (AK8963)

Append a row `[0, 1, 0, 0, 0, 0]` to H, append `atan2(m_y, m_x)` to z (after hard/soft-iron calibration on the robot — the ±300 µT factory tolerance makes raw data useless otherwise), and append σ²_mag ≈ (0.02~0.05)² rad² to R (tune from indoor measurements).

Enable only in clean magnetic environments: gate with `y² > 9·S_mag` → discard that update (motor currents and steel structure make the robot a hostile place for a magnetometer).

---

## 7. Tuning Procedure

1. **Stationary IMU log** (≥10 min): Allan variance → IMU terms of R, Q_bg, Q_ba.
2. **Steady-speed wheel logs**: several speeds → per-wheel σ²_ω; verify 1/Δt scaling; cross-check wheel radius/wheelbase against the gyro.
3. **Power-on stationary calibration**: estimate b_g_cal, b_a_cal, load into x₀.
4. **Full-vehicle test**: drive 10 m straight, rotate 360° in place; compare filter output against an external ground-truth pose; scale Q_v, Q_ω, and encoder R according to residual ratios.

---

## Appendix: Minimal Runnable Parameters

```javascript
Δt = 0.01 s (100 Hz IMU), encoder window 0.02–0.05 s
R = diag(σ²_ωL, σ²_ωR, 1.5e-6)          ← encoder entries from measurement
Q = diag(0, 0, (0.05Δt)², (0.01Δt)², 1e-10, 1e-6Δt)
x₀ = [0, 0, 0, 0, b_g_cal, b_a_cal]
P₀ = diag(0, 0, 1, 1, 7.6e-3, 0.615)
```