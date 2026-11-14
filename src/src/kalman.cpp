#include "kalman.h"

#include <math.h>

namespace
{
constexpr float KF_PI = 3.14159265358979f;
constexpr float KF_TWO_PI = 2.0f * KF_PI;
} // namespace

// Fold into (-pi, pi].
float wrapAngle(float a)
{
  a = fmodf(a, KF_TWO_PI);
  if (a <= -KF_PI)
    a += KF_TWO_PI;
  else if (a > KF_PI)
    a -= KF_TWO_PI;
  return a;
}

void StaticDecoupledEstimator::reset()
{
  v = 0.0f;
  omega = 0.0f;
  b_a = 0.0f;
  for (int i = 0; i < KF_MEAS_COUNT; ++i)
  {
    lastInnovation[i] = 0.0f;
    lastInnovationValid[i] = false;
  }
  healthy = true;
}

bool StaticDecoupledEstimator::predict(float aForward, float dt)
{
  if (!isfinite(aForward) || !isfinite(dt) || dt <= 0.0f || dt > 0.05f)
    return false;

  // v' = v + (a - b_a) * dt.
  v += (aForward - b_a) * dt;

  healthy = healthy && finiteState();
  return healthy;
}

bool StaticDecoupledEstimator::updateGyro(float gyroZ)
{
  if (!isfinite(gyroZ))
  {
    lastInnovationValid[KF_M_GYRO] = false;
    return false;
  }

  // H*x = omega. The driver has already removed the gyro offset.
  const float innovation = gyroZ - omega;
  omega += kf_gains::K_GYRO * innovation;

  lastInnovation[KF_M_GYRO] = innovation;
  lastInnovationValid[KF_M_GYRO] = true;
  healthy = healthy && finiteState();
  return healthy;
}

bool StaticDecoupledEstimator::updateWheelV(float vMeas)
{
  if (!isfinite(vMeas))
  {
    lastInnovationValid[KF_M_WHEEL_V] = false;
    return false;
  }

  // H*x = v.
  const float innovation = vMeas - v;
  v += kf_gains::K_WHEEL_V[0] * innovation;
  b_a += kf_gains::K_WHEEL_V[1] * innovation;

  lastInnovation[KF_M_WHEEL_V] = innovation;
  lastInnovationValid[KF_M_WHEEL_V] = true;
  healthy = healthy && finiteState();
  return healthy;
}

bool StaticDecoupledEstimator::updateWheelOmega(float omegaMeas)
{
  if (!isfinite(omegaMeas))
  {
    lastInnovationValid[KF_M_WHEEL_OMEGA] = false;
    return false;
  }

  // H*x = omega.
  const float innovation = omegaMeas - omega;
  omega += kf_gains::K_WHEEL_OMEGA * innovation;

  lastInnovation[KF_M_WHEEL_OMEGA] = innovation;
  lastInnovationValid[KF_M_WHEEL_OMEGA] = true;
  healthy = healthy && finiteState();
  return healthy;
}

bool StaticDecoupledEstimator::finiteState() const
{
  return isfinite(v) && isfinite(omega) && isfinite(b_a);
}

StaticDecoupledEstimator estimator;
