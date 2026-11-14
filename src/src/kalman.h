#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>

// Fold an angle into (-pi, pi].
float wrapAngle(float a);

// Wheel-velocity differencing window used by the estimator updates (s).
constexpr float KF_WHEEL_WINDOW = 0.010f;

// Index into StaticDecoupledEstimator::lastInnovation[]. The reserved entry
// keeps the serial-log column positions stable; it is never valid.
enum KfMeasurement : uint8_t
{
  KF_M_GYRO = 0,
  KF_M_WHEEL_V = 1,
  KF_M_WHEEL_OMEGA = 2,
  KF_M_RESERVED = 3,
  KF_MEAS_COUNT = 4,
};

// Constant gains for the decoupled stationary filter. Each entry maps one
// scalar innovation into its state block. Values come from the block DARE in
// kalman_filter_tuning/solve_dare.m; regenerate them when its parameters
// change.
//
// Wheel v update (100 Hz),  over [v, b_a]
// Gyro update     (1 kHz),   over [omega]
// Wheel omega     (100 Hz),  over [omega]
namespace kf_gains
{
constexpr float K_WHEEL_V[2] = {0.197604345f, -0.018533077f};
constexpr float K_GYRO = 0.209729549f;
constexpr float K_WHEEL_OMEGA = 0.012222362f;
} // namespace kf_gains

/**
 * Stationary, decoupled estimator for a planar differential-drive robot.
 *
 * State: [v, omega, b_a]
 *   v     forward velocity (m/s)
 *   omega yaw rate (rad/s)
 *   b_a   residual accelerometer bias (m/s^2)
 *
 * Heading, distance, x and y are deliberately not filter states. The firmware
 * integrates them outside the filter from v and omega, so the filter remains
 * time-invariant and its gains are constant.
 *
 * The gyro bias is removed by the IMU driver's startup offset calibration;
 * it is not an estimator state. The magnetometer is not fused.
 *
 * No covariance is propagated. Measurement noise is folded into the
 * precomputed gains, and each update is a fixed linear correction.
 *
 * No Arduino headers, so the module also builds on the host.
 */
struct StaticDecoupledEstimator
{
  float v = 0.0f;     // forward velocity (m/s)
  float omega = 0.0f; // yaw rate (rad/s)
  float b_a = 0.0f;   // accelerometer bias (m/s^2)

  // Last scalar innovation y = z - H*x per measurement type. Invalid updates
  // leave the value and clear lastInnovationValid.
  float lastInnovation[KF_MEAS_COUNT] = {};
  bool lastInnovationValid[KF_MEAS_COUNT] = {};
  bool healthy = true;

  void reset();

  // Propagate v. The accelerometer reading is raw: b_a is subtracted here.
  // Heading and position integration happen outside the filter.
  bool predict(float aForward, float dt);

  // Measurements. Each returns false when the input is not finite.
  bool updateGyro(float gyroZ);
  bool updateWheelV(float vMeas);
  bool updateWheelOmega(float omegaMeas);

  float velocity() const { return v; }
  float yawRate() const { return omega; }
  float accelBias() const { return b_a; }

private:
  bool finiteState() const;
};

extern StaticDecoupledEstimator estimator;

#endif
