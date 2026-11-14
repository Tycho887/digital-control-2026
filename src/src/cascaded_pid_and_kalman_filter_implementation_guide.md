# Implementation Guide: Cascaded PID & Multi-Rate State Filter

## 1. System Architecture Overview

```
                   +--------------------+
  v_ref (m/s) ---->|  forwardVelPID     |---+
                   +--------------------+   | u_v      +-----------------+   w_L_ref   +---------------+
                               ^            +--------->|                 |------------>| leftWheelPID  |---> V_L
                               | v_est                 | Kinematic Mixer |             +---------------+
                   +--------------------+   +--------->|  (Diff-Drive)   |             +---------------+
  w_ref (rad/s) -->|   turnRatePID      |---+          |                 |------------>| rightWheelPID |---> V_R
                   +--------------------+     u_w      +-----------------+   w_R_ref   +---------------+
                               ^                                                          ^         ^
                               | w_est                                                    |         |
                   +-----------------------------------------------------+                |         |
  IMU / Encoders --->|        DiffDriveKalmanFilter (Continuous-Discrete)  |----------------+---------+
  (AS5 optional)     +-----------------------------------------------------+     w_L_est, w_R_est (or AlphaBeta)
```

---

## 2. Step-by-Step Code Changes

### Step 1: Unify `PID` and `WheelController` (`basebot_6.ino`)
Replace the empty `class PID` and `class WheelController` with a single `PIDController` class that supports both pre-calculated discrete difference equations (`PID_Coefficients`) and standard continuous gains $(K_p, K_i, K_d)$.

* **Action:** Replace `PID` and `WheelController` definitions with:
  ```cpp
  class PIDController {
    public:
      PID_Coefficients coeffs;
      float u1 = 0.0f, u2 = 0.0f, e1 = 0.0f, e2 = 0.0f;
      float uMin = -3.0f, uMax = 3.0f;

      PIDController() = default;
      PIDController(PID_Coefficients c, float minOut, float maxOut)
        : coeffs(c), uMin(minOut), uMax(maxOut) {}

      void setGainsPID(float Kp, float Ki, float Kd, float Ts, float minOut, float maxOut) {
        coeffs.b0 = Kp + Ki * Ts + (Kd / Ts);
        coeffs.b1 = -Kp - (2.0f * Kd / Ts);
        coeffs.b2 = Kd / Ts;
        coeffs.a1 = -1.0f;
        coeffs.a2 = 0.0f;
        uMin = minOut;
        uMax = maxOut;
      }

      void reset() { u1 = u2 = e1 = e2 = 0.0f; }

      float update(float setpoint, float measurement) {
        float err = setpoint - measurement;
        float u = (coeffs.b0 * err) + (coeffs.b1 * e1) + (coeffs.b2 * e2)
                  - (coeffs.a1 * u1) - (coeffs.a2 * u2);
        if (u > uMax) u = uMax;
        else if (u < uMin) u = uMin;
        e2 = e1; e1 = err;
        u2 = u1; u1 = u;
        return u;
      }
  };
  ```
* **Action:** Instantiate all four controllers from `PIDController`:
  ```cpp
  PIDController leftController(left_wheel_coeffs, -controlVoltageLimit, controlVoltageLimit);
  PIDController rightController(right_wheel_coeffs, -controlVoltageLimit, controlVoltageLimit);
  PIDController forwardVelController;
  PIDController turnRateController;
  ```

---

### Step 2: Add Individual Wheel Updates & Projections to KF (`kalman.h` & `kalman.cpp`)
Instead of adding redundant wheel velocity states to the state vector $x = [v, \omega, b_a]^T$ (which makes covariance $P$ singular), project $(v, \omega)$ onto individual wheels during measurement updates and state readouts. Heading and distance are integrated outside the filter; the magnetometer is not fused.

* **Action (`kalman.h`):** Declare individual wheel update and projection methods in `DiffDriveKalmanFilter`:
  ```cpp
  void updateLeftWheel(float vLeft, float window);
  void updateRightWheel(float vRight, float window);
  void getWheelMotorVel(float &leftRadPerSec, float &rightRadPerSec) const;
  ```
* **Action (`kalman.cpp`):** Implement the scalar updates using the kinematic observation rows $H_L = [0, 0, 1, -B/2, 0, 0, 0]$ and $H_R = [0, 0, 1, +B/2, 0, 0, 0]$:
  ```cpp
  void DiffDriveKalmanFilter::updateLeftWheel(float vLeft, float window) {
    const float H[KF_N] = {1.0f, -0.5f * KF_WHEEL_BASE, 0.0f};
    updateScalar(H, vLeft, 2.0f * rWheelV(window), M_WHEEL_V);
  }

  void DiffDriveKalmanFilter::updateRightWheel(float vRight, float window) {
    const float H[KF_N] = {1.0f, +0.5f * KF_WHEEL_BASE, 0.0f};
    updateScalar(H, vRight, 2.0f * rWheelV(window), M_WHEEL_OMEGA);
  }

  void DiffDriveKalmanFilter::getWheelMotorVel(float &leftRadPerSec, float &rightRadPerSec) const {
    const float vLeft  = x[V] - 0.5f * KF_WHEEL_BASE * x[OMEGA];
    const float vRight = x[V] + 0.5f * KF_WHEEL_BASE * x[OMEGA];
    leftRadPerSec  = vLeft  * (KF_AS5X47_GEAR / KF_WHEEL_RADIUS);
    rightRadPerSec = vRight * (KF_AS5X47_GEAR / KF_WHEEL_RADIUS);
  }
  ```

---

### Step 3: Wire the Cascaded Control Loop (`basebot_6.ino`)
Update `setup()`, `start()`, and `controlUpdate()` to run the outer sum-velocity and turn-rate loops feeding the inner wheel loops.

* **Action (`setup()`):** Configure initial gains and saturation limits for the outer loops:
  ```cpp
  const float Ts = sampleTimeUs * 1e-6f;
  forwardVelController.setGainsPID(0.8f, 4.0f, 0.0f, Ts, -0.3f, 0.3f); // Trim in m/s
  turnRateController.setGainsPID(1.2f, 6.0f, 0.0f, Ts, -2.5f, 2.5f);   // Trim in rad/s
  ```
* **Action (`start()` & `resetWheelControl()`):** Reset the outer controllers alongside the inner controllers:
  ```cpp
  forwardVelController.reset();
  turnRateController.reset();
  ```
* **Action (`controlUpdate()`):** Compute the outer loop commands, mix into left/right motor references, and execute `wheelSpeedControl()`:
  ```cpp
  float desiredTurnRate = 0.0f; // Set by mission sequence (rad/s)

  void controlUpdate() {
    // 1. Read fused robot velocity & yaw rate from Kalman Filter
    float vEst     = kf.x[DiffDriveKalmanFilter::V];
    float omegaEst = kf.x[DiffDriveKalmanFilter::OMEGA];

    // 2. Outer Loops: Feedforward + PI Trim
    float vCmd     = desiredValue    + forwardVelController.update(desiredValue, vEst);
    float omegaCmd = desiredTurnRate + turnRateController.update(desiredTurnRate, omegaEst);

    // 3. Differential Drive Mixer -> Motor Shaft Setpoints (rad/s)
    float vLeftRef  = vCmd - 0.5f * wheelBase * omegaCmd;
    float vRightRef = vCmd + 0.5f * wheelBase * omegaCmd;

    float leftMotorRef  = vLeftRef  * (gear / wheelRadius);
    float rightMotorRef = vRightRef * (gear / wheelRadius);

    // 4. Inner Loops: Individual Wheel Velocity Controllers
    wheelSpeedControl(leftMotorRef, rightMotorRef);
  }
  ```

---

## 3. Tuning Order Checklist
1. **Inner Wheel Loops First (`leftController`, `rightController`):** Tune for fast response ($\approx 15\text{--}25\text{ Hz}$ bandwidth) using `AlphaBetaFilter` encoder feedback so asymmetric internal motor resistances are rejected immediately.
2. **State Filter Verification (`DiffDriveKalmanFilter`):** Run a test sequence and verify in the logs that `kfState[V]` and `kfState[OMEGA]` smoothly track encoder and IMU measurements without phase lag.
3. **Outer Robot Loops (`forwardVelController`, `turnRateController`):** Start with pure proportional gain ($K_i = 0, K_d = 0$) at a bandwidth $5\times$ to $10\times$ slower than the inner wheel loops ($\approx 2\text{--}4\text{ Hz}$), then add $K_i$ to eliminate steady-state velocity and heading drift over your 10-second horizon.