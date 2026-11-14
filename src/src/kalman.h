#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>

// Fold an angle into (-pi, pi].
float wrapAngle(float a);

// AS5X47 angle-differencing window used for the wheel velocity updates (s).
constexpr float KF_WHEEL_WINDOW = 0.010f;

// Index into StaticDecoupledEstimator::lastInnovation[].
enum KfMeasurement : uint8_t
{
  KF_M_GYRO = 0,
  KF_M_WHEEL_V = 1,
  KF_M_WHEEL_OMEGA = 2,
  KF_M_MAG = 3,
  KF_MEAS_COUNT = 4,
};

// Constant gains for the decoupled stationary filter. Each row maps one scalar
// innovation into its state block. The values are the 10 ms wheel / 1 kHz
// gyro / 10 Hz magnetometer bank; the firmware throttles the 100 Hz AK8963
// samples to 10 Hz before calling updateMag(). Regenerate them from the
// stationary DARE when its parameters change.
//
// Wheel v update (100 Hz),   H = [1 0]     over [v, b_a]
// Gyro update     (1 kHz),   H = [0 1 1]   over [theta, omega, b_g]
// Wheel omega     (100 Hz),  H = [0 1 0]   over [theta, omega, b_g]
// Magnetometer    (async),   H = [1 0 0]   over [theta, omega, b_g]
namespace kf_gains
{
constexpr float K_WHEEL_V[2] = {0.18f, -0.35f};
constexpr float K_GYRO[3] = {0.002f, 0.85f, 0.0005f};
constexpr float K_WHEEL_OMEGA[3] = {0.01f, 0.12f, -0.04f};
constexpr float K_MAG[3] = {0.05f, 0.0f, -0.002f};
constexpr float MAG_GATE_RAD = 0.25f; // static outlier gate (~14 deg)
} // namespace kf_gains

/**
 * Stationary, decoupled estimator for a planar differential-drive robot.
 *
 * Translational state: [v, b_a]
 * Rotational state:    [theta, omega, b_g]
 *
 * Position (s, x, y) is deliberately not a filter state. The firmware
 * dead-reckons it from v and theta after each predict, so the filter remains
 * time-invariant and its gains are constant.
 *
 * No covariance is propagated. Measurement noise is folded into the
 * precomputed gains, and each update is a fixed linear correction.
 *
 * No Arduino headers, so the module also builds on the host.
 */
struct StaticDecoupledEstimator
{
  // Translational state.
  float v = 0.0f;   // forward velocity (m/s)
  float b_a = 0.0f; // accelerometer bias (m/s^2)

  // Rotational state.
  float theta = 0.0f; // heading (rad)
  float omega = 0.0f; // yaw rate (rad/s)
  float b_g = 0.0f;   // gyro bias (rad/s)

  // Last scalar innovation y = z - H*x per measurement type. Rejected or
  // invalid updates leave the value and clear lastInnovationValid.
  float lastInnovation[KF_MEAS_COUNT] = {};
  bool lastInnovationValid[KF_MEAS_COUNT] = {};
  bool healthy = true;

  void reset(float heading0 = 0.0f);

  // Propagate v and theta. The accelerometer reading is raw: b_a is
  // subtracted here. Position integration happens outside the filter.
  bool predict(float aForward, float dt);

  // Measurements. Each returns false when the input is not finite or, for
  // the magnetometer, when the innovation gate rejects the sample.
  bool updateGyro(float gyroZ);
  bool updateWheelV(float vMeas);
  bool updateWheelOmega(float omegaMeas);
  bool updateMag(float magHeading);

  float velocity() const { return v; }
  float yawRate() const { return omega; }
  float heading() const { return theta; }
  float accelBias() const { return b_a; }
  float gyroBias() const { return b_g; }

private:
  bool finiteState() const;
};

extern StaticDecoupledEstimator estimator;

#endif
