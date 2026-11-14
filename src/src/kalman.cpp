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

void StaticDecoupledEstimator::reset(float heading0)
{
  v = 0.0f;
  b_a = 0.0f;
  theta = wrapAngle(heading0);
  omega = 0.0f;
  b_g = 0.0f;
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

  // v' = v + (a - b_a) * dt, theta' = theta + omega * dt.
  v += (aForward - b_a) * dt;
  theta = wrapAngle(theta + omega * dt);

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

  // H*x = omega + b_g.
  const float innovation = gyroZ - (omega + b_g);
  theta = wrapAngle(theta + kf_gains::K_GYRO[0] * innovation);
  omega += kf_gains::K_GYRO[1] * innovation;
  b_g += kf_gains::K_GYRO[2] * innovation;

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
  theta = wrapAngle(theta + kf_gains::K_WHEEL_OMEGA[0] * innovation);
  omega += kf_gains::K_WHEEL_OMEGA[1] * innovation;
  b_g += kf_gains::K_WHEEL_OMEGA[2] * innovation;

  lastInnovation[KF_M_WHEEL_OMEGA] = innovation;
  lastInnovationValid[KF_M_WHEEL_OMEGA] = true;
  healthy = healthy && finiteState();
  return healthy;
}

bool StaticDecoupledEstimator::updateMag(float magHeading)
{
  if (!isfinite(magHeading))
  {
    lastInnovationValid[KF_M_MAG] = false;
    return false;
  }

  // H*x = theta.
  const float innovation = wrapAngle(magHeading - theta);
  lastInnovation[KF_M_MAG] = innovation;
  if (fabsf(innovation) > kf_gains::MAG_GATE_RAD)
  {
    // Magnetic disturbance: keep the sample out of the state.
    lastInnovationValid[KF_M_MAG] = false;
    return false;
  }

  theta = wrapAngle(theta + kf_gains::K_MAG[0] * innovation);
  omega += kf_gains::K_MAG[1] * innovation;
  b_g += kf_gains::K_MAG[2] * innovation;

  lastInnovationValid[KF_M_MAG] = true;
  healthy = healthy && finiteState();
  return healthy;
}

bool StaticDecoupledEstimator::finiteState() const
{
  return isfinite(v) && isfinite(b_a) && isfinite(theta) &&
         isfinite(omega) && isfinite(b_g);
}

StaticDecoupledEstimator estimator;
