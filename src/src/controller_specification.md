# Controller Specification — Cascaded Polar & Wheel-Speed Control

Discrete-time control architecture (`basebot_6.ino`, `pid.h`, `trajectory.h`) for the Teensy 4.1 differential-drive robot, executing deterministically at $1\text{ kHz}$ ($T_s = 0.001\text{ s}$).

---

## 1. Control Architecture Overview

The control system is structured as a two-level cascade separating chassis-level polar motion $(v, \omega)$ from inner motor-shaft speed regulation $(\omega_{m,L}, \omega_{m,R})$:

```
Trajectory ──► v_ref ────► [velLimiter] ─────────────────────────────────────┐
                                                                             ▼
               psi_ref ──► ( + ) ──► [headingPid] ──► ( + ) ──► [turnLimiter] ──► ( + ) ──► [polar2Cartesian] ──► Inner Wheel PIs
                             ▲                          ▲                           ▲
               omega_ref ────|──────────────────────────┘                           │
                             │                                                      │
       dead-reckoned theta ─┘ (when useKfFeedback = true)                          │
               KF (omega) ────────────────────────────────────────► [yawRatePid] ───┘
```

* **Closed-Loop vs. Feedforward Mode (`useKfFeedback`):**
  * `useKfFeedback = true`: Heading closes around the dead-reckoned `poseHeading`; the yaw-rate loop closes around `estimator.omega`. The estimator state is `[v, omega, b_a]`; heading and distance are integrated outside the filter.
  * `useKfFeedback = false`: Outer heading and yaw-rate errors are zeroed, operating purely as rate-limited feedforward into the $1\text{ kHz}$ inner wheel-speed PI loops.
* **Forward Velocity Path:** No outer velocity PID is used. Reference-limited velocity $v_{\text{cmd}}$ feeds directly into `polar2Cartesian()`, relying on the high-bandwidth inner wheel-speed PI controllers for linear velocity tracking.

---

## 2. Discrete-Time Compensator Form (`PIDController`)

All linear controllers share a common 2nd-order IIR difference equation (`ComputeCCDE(error)`), discretized from continuous MATLAB designs via the Tustin (bilinear) transform at $T_s = 1\text{ ms}$:

$$u[k] = b_0 e[k] + b_1 e[k-1] + b_2 e[k-2] - a_1 u[k-1] - a_2 u[k-2]$$

Output clamping is applied at each step ($|u[k]| \le u_{\max}$), and the clamped value is stored in $u[k-1]$ to prevent integrator windup.

---

## 3. Reference Rate Limiters (`ReferenceLimiter`)

Step references are shaped by first-order rate-saturated pre-filters prior to entering the control loops:

$$y[k] = y[k-1] + T_s \cdot \operatorname{sat}\!\left(K_p (r[k] - y[k-1]),\; \dot{y}_{\min},\; \dot{y}_{\max}\right)$$

| Limiter Instance | Signal | Gain $K_p$ | Lower Rate Limit | Upper Rate Limit |
| :--- | :--- | :--- | :--- | :--- |
| `velLimiter` | Forward speed $v_{\text{ref}}$ ($\text{m/s}$) | $25.0\text{ s}^{-1}$ | $-0.5\text{ m/s}^2$ | $+0.5\text{ m/s}^2$ |
| `turnLimiter` | Turn rate $\omega_{\text{ref}} + \Delta\omega_\psi$ ($\text{rad/s}$) | $25.0\text{ s}^{-1}$ | $-0.7\text{ rad/s}^2$ | $+0.7\text{ rad/s}^2$ |

---

## 4. Outer-Loop Controllers (Chassis Polar Coordinates)

### A. Heading Lead-Lag Controller (`headingPid`)
* **Purpose:** Eliminates accumulated heading drift by generating a corrective yaw-rate offset $\Delta\omega_\psi$ ($\text{rad/s}$).
* **Error Input:** $e_\psi[k] = \operatorname{wrap}(\psi_{\text{ref}}[k] - \hat{\theta}[k])$ ($\text{rad}$).
* **Output Saturation:** $\pm \text{Lim}_{TU} = \pm 0.7\text{ rad/s}$.
* **Coefficients ($T_s = 1\text{ ms}$, includes proportional gain $K_{ph} = 2.5$):**

| $b_0$ | $b_1$ | $b_2$ | $a_1$ | $a_2$ |
| :--- | :--- | :--- | :--- | :--- |
| `4.996878901f` | `-4.990636704f` | `0.0f` | `-0.997503121f` | `0.0f` |

### B. Yaw-Rate PI Controller (`yawRatePid`)
* **Purpose:** Tracks the rate-limited turn reference $\omega_{\text{ref,lim}}[k] = \text{turnLimiter}(\omega_{\text{ref}}[k] + \Delta\omega_\psi[k])$.
* **Error Input:** $e_\omega[k] = \omega_{\text{ref,lim}}[k] - \hat{\omega}[k]$ ($\text{rad/s}$).
* **Output Saturation:** $\pm \text{Lim}_{TU} = \pm 0.7\text{ rad/s}$.
* **Total Yaw-Rate Command:** $\omega_{\text{cmd}}[k] = \omega_{\text{ref,lim}}[k] + \Delta\omega_\omega[k]$.
* **Coefficients ($T_s = 1\text{ ms}$, $K_{pt} = 5.0$, $\tau_{i,t} = 0.2\text{ s}$):**

| $b_0$ | $b_1$ | $b_2$ | $a_1$ | $a_2$ |
| :--- | :--- | :--- | :--- | :--- |
| `5.0125f` | `-4.9875f` | `0.0f` | `-1.0f` | `0.0f` |

---

## 5. Differential-Drive Inverse Kinematics (`polar2Cartesian`)

Chassis commands $(v_{\text{cmd}}, \omega_{\text{cmd}})$ are mapped to left and right wheel linear velocities ($\text{m/s}$) and scaled to motor-shaft angular velocity setpoints ($\text{rad/s}$):

$$v_L = v_{\text{cmd}} - \frac{b}{2}\omega_{\text{cmd}}, \qquad v_R = v_{\text{cmd}} + \frac{b}{2}\omega_{\text{cmd}}$$
$$\omega_{m,L,\text{ref}} = v_L \frac{G}{r_w}, \qquad \omega_{m,R,\text{ref}} = v_R \frac{G}{r_w}$$

| Parameter | Symbol | Value | Unit |
| :--- | :--- | :--- | :--- |
| Gear ratio | $G$ | $9.6$ | — |
| Wheel radius | $r_w$ | $0.0315$ | $\text{m}$ |
| Wheel base (track width) | $b$ | $0.14$ | $\text{m}$ |

---

## 6. Inner-Loop Wheel Velocity Estimation & PI Control

### A. Quadrature Encoder Alpha-Beta Velocity Filter (`AlphaBetaFilter`)
Motor-shaft speed feedback is estimated at $1\text{ kHz}$ from 48 PPR quadrature encoders (`leftCounts = -encoder[0]`, `rightCounts = encoder[1]`):

$$\hat{p}_{\text{pred}} = \hat{p}_{k-1} + \hat{v}_{k-1} T_s, \qquad r = p_{\text{meas}} - \hat{p}_{\text{pred}}$$
$$\hat{p}_k = \hat{p}_{\text{pred}} + \alpha r, \qquad \hat{v}_k = \hat{v}_{k-1} + \frac{\beta}{T_s} r, \qquad \omega_{m,\text{meas}} = \hat{v}_k \frac{2\pi}{48}$$

| Filter Instance | $\alpha$ | $\beta$ | $T_s$ |
| :--- | :--- | :--- | :--- |
| `leftVelFilter` | $0.5$ | $0.1$ | $0.001\text{ s}$ |
| `rightVelFilter` | $0.8$ | $0.2$ | $0.001\text{ s}$ |

### B. Inner Wheel-Speed PI Controllers (`leftController`, `rightController`)
* **Error Input:** $e_{m,i}[k] = \omega_{m,i,\text{ref}}[k] - \omega_{m,i,\text{meas}}[k]$ ($\text{rad/s}$).
* **Output Saturation:** $\pm \text{controlVoltageLimit} = \pm 3.0\text{ V}$ (applied to H-bridge at $80\text{ kHz}$ PWM with `motorVoltage[0] = -u_L`, `motorVoltage[1] = +u_R`).
* **Coefficients ($T_s = 1\text{ ms}$):**

| Controller | $b_0$ | $b_1$ | $b_2$ | $a_1$ | $a_2$ |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `leftController` | `0.05005f` | `-0.04995f` | `0.0f` | `-1.0f` | `0.0f` |
| `rightController` | `0.04005f` | `-0.03995f` | `0.0f` | `-1.0f` | `0.0f` |

---

## 7. Trajectory Generation & Lifecycle

* **Default Closed-Loop Trajectory (`closedLoopTrajectory()` in `trajectory.h`):**
  1. $0.2\text{ s}$ stationary baseline ($v_{\text{ref}} = 0$, $\omega_{\text{ref}} = 0$).
  2. $4.0\text{ s}$ straight segment at $v_{\text{ref}} = 0.20\text{ m/s}$, $\omega_{\text{ref}} = 0$.
  3. $3.0\text{ s}$ in-place turn at $v_{\text{ref}} = 0$, $\omega_{\text{ref}} = +30^\circ/\text{s}$ ($0.5236\text{ rad/s}$), advancing $\psi_{\text{ref}}$ along `controlOmegaRefLimited * Ts` so `headingPid` does not oppose rate-limiter ramp-up.
  4. $0.4\text{ s}$ zero-command settling phase.
* **Initialization & Shutdown:** `start()` resets all `PIDController`, `ReferenceLimiter`, and `AlphaBetaFilter` states to zero and locks $\psi_{\text{ref}}$ to the current estimator heading. `finished()` forces zero motor voltage immediately before serial log transfer.

---

## 8. MATLAB Coefficient Export

To generate `PID_Coefficients` for any continuous-time controller $C(s)$ at $T_s = 1\text{ ms}$:

```matlab
Ts = 0.001;
C_disc = c2d(C_cont, Ts, 'tustin');
[num, den] = tfdata(C_disc, 'v');
fprintf('PID_Coefficients coeffs = {%.9ff, %.9ff, %.9ff, %.9ff, %.9ff};\n', ...
        num(1), num(2), num(3), den(2), den(3));
```