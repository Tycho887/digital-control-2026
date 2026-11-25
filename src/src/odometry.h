#ifndef BASEBOT_ODOMETRY_H
#define BASEBOT_ODOMETRY_H

#include <math.h>

struct OdometryIncrement
{
  float distance;
  float yaw;
};

// Integrate sampled forward speed and yaw rate over disjoint pairs of ticks.
// The caller applies the usual trapezoid each tick, plus the correction below.
class PairedSimpson
{
public:
  OdometryIncrement correction(float dt, float changeInSpeed, float changeInYawRate)
  {
    if (firstDt == 0.0f)
    {
      firstDt = dt;
      firstSpeedChange = changeInSpeed;
      firstYawRateChange = changeInYawRate;
      return {};
    }

    const float a = firstDt;
    const float b = dt;
    discardPendingInterval();
    // Beyond a 2:1 interval ratio the quadratic rule has a negative endpoint
    // weight. Keep both trapezoids when scheduler jitter exceeds that ratio.
    if (a > 2.0f * b || b > 2.0f * a) return {};

    // Integral of the interpolating quadratic minus the two trapezoids:
    // (a² - ab + b²) / 6 times (first slope - second slope).
    const float factor = (a * a - a * b + b * b) / 6.0f;
    const float firstWeight = factor / a;
    const float secondWeight = factor / b;
    const OdometryIncrement delta = {
      firstWeight * firstSpeedChange - secondWeight * changeInSpeed,
      firstWeight * firstYawRateChange - secondWeight * changeInYawRate
    };
    distanceDifference += delta.distance;
    yawDifference += delta.yaw;
    peakDistanceDifference = fmax(peakDistanceDifference, fabs(distanceDifference));
    peakYawDifference = fmax(peakYawDifference, fabs(yawDifference));
    return delta;
  }

  void discardPendingInterval() { firstDt = 0.0f; }

  // Cumulative differences on identical KF samples, in metres and radians.
  double distanceDifference = 0.0, yawDifference = 0.0;
  double peakDistanceDifference = 0.0, peakYawDifference = 0.0;

private:
  float firstDt = 0.0f;
  float firstSpeedChange = 0.0f;
  float firstYawRateChange = 0.0f;
};

#endif
