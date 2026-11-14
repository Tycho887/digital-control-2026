// Host-side checks for the stationary decoupled estimator.
//
// Build and run from the project root:
//   c++ -std=c++14 -O2 -Isrc/src tests/kalman_host/check.cpp src/src/kalman.cpp \
//       -o /tmp/kalman-check
#include "kalman.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static const float PI_F = 3.14159265358979f;

static bool near(float a, float b, float tol)
{
  return fabsf(a - b) <= tol;
}

static bool allFinite(const StaticDecoupledEstimator &f)
{
  return isfinite(f.v) && isfinite(f.b_a) && isfinite(f.omega);
}

static void testWrapAngle()
{
  assert(near(wrapAngle(0.0f), 0.0f, 1e-6f));
  assert(near(wrapAngle(0.5f), 0.5f, 1e-6f));
  assert(near(wrapAngle(-0.5f), -0.5f, 1e-6f));
  assert(near(wrapAngle(PI_F), PI_F, 1e-5f));
  assert(near(wrapAngle(-PI_F), PI_F, 1e-5f));
  assert(near(wrapAngle(1.5f * PI_F), -0.5f * PI_F, 1e-5f));
  assert(near(wrapAngle(-1.5f * PI_F), 0.5f * PI_F, 1e-5f));
  assert(near(wrapAngle(4.0f * PI_F + 0.3f), 0.3f, 1e-5f));

  const float samples[10] = {7.0f, -7.0f, 10.0f, -10.0f, 100.0f,
                             -100.0f, 3.0f, -3.0f, 6.0f, -6.0f};
  for (int i = 0; i < 10; ++i)
  {
    float w = wrapAngle(samples[i]);
    assert(w > -PI_F);
    assert(w <= PI_F);
  }
  printf("PASS wrapAngle range and values\n");
}

static void testPredictKinematics()
{
  StaticDecoupledEstimator f;
  f.reset();
  for (int i = 0; i < 1000; ++i)
    assert(f.predict(0.5f, 0.001f));
  assert(near(f.velocity(), 0.5f, 1e-3f));
  assert(near(f.accelBias(), 0.0f, 1e-6f));
  assert(near(f.yawRate(), 0.0f, 1e-6f));
  assert(f.healthy);
  printf("PASS predict from rest: v=%.4f\n", f.velocity());

  assert(!f.predict(1.0f, 0.0f));
  assert(!f.predict(1.0f, 0.051f));
  assert(!f.predict(1.0f, -0.001f));
  assert(!f.predict(NAN, 0.001f));
  assert(!f.predict(INFINITY, 0.001f));
  assert(!f.predict(1.0f, NAN));
  printf("PASS predict input validation\n");
}

static void testAccelerometerTracking()
{
  // Constant true acceleration; wheel v arrives every 10 ms. The filter
  // should track velocity without absorbing the acceleration into b_a.
  StaticDecoupledEstimator f;
  f.reset();
  const float a = 0.5f;
  float trueV = 0.0f;
  for (int i = 0; i < 3000; ++i)
  {
    trueV += a * 0.001f;
    assert(f.predict(a, 0.001f));
    if ((i % 10) == 0)
      assert(f.updateWheelV(trueV));
  }
  assert(near(f.velocity(), trueV, 2e-3f));
  assert(fabsf(f.accelBias()) < 2e-3f);
  assert(f.healthy);
  printf("PASS accel tracking: v=%.4f true=%.4f b_a=%.5f\n",
         f.velocity(), trueV, f.accelBias());
}

static void testAccelerometerBias()
{
  // Stationary robot with a raw accelerometer offset of +0.2 m/s^2. The
  // wheel v measurement is zero, so b_a must converge to the offset. The
  // DARE bias gain is deliberately slow, so run longer than the old check.
  StaticDecoupledEstimator f;
  f.reset();
  const float rawAccel = 0.2f;
  for (int i = 0; i < 60000; ++i)
  {
    assert(f.predict(rawAccel, 0.001f));
    if ((i % 10) == 0)
      assert(f.updateWheelV(0.0f));
  }
  assert(near(f.velocity(), 0.0f, 1e-3f));
  assert(near(f.accelBias(), rawAccel, 1e-3f));
  assert(f.healthy);
  printf("PASS accel bias: b_a=%.5f (true %.2f)\n",
         f.accelBias(), rawAccel);
}

static void testGyroAndWheelOmega()
{
  // Constant true yaw rate. The gyro update runs every step; the wheel yaw
  // rate runs every 10 ms.
  StaticDecoupledEstimator f;
  f.reset();
  const float trueOmega = 0.5f;
  const float dt = 0.001f;
  for (int i = 0; i < 3000; ++i)
  {
    assert(f.predict(0.0f, dt));
    assert(f.updateGyro(trueOmega));
    if ((i % 10) == 0)
      assert(f.updateWheelOmega(trueOmega));
  }
  assert(near(f.yawRate(), trueOmega, 2e-3f));
  assert(f.healthy);
  printf("PASS gyro/wheel omega: omega=%.5f\n", f.yawRate());
}

static void testReset()
{
  StaticDecoupledEstimator f;
  f.reset();
  assert(near(f.velocity(), 0.0f, 1e-6f));
  assert(near(f.yawRate(), 0.0f, 1e-6f));
  assert(near(f.accelBias(), 0.0f, 1e-6f));
  assert(f.healthy);
  for (int i = 0; i < KF_MEAS_COUNT; ++i)
    assert(!f.lastInnovationValid[i]);
  printf("PASS reset state\n");
}

static void testInvalidMeasurements()
{
  StaticDecoupledEstimator f;
  f.reset();
  assert(!f.updateGyro(NAN));
  assert(!f.updateWheelV(INFINITY));
  assert(!f.updateWheelOmega(NAN));
  assert(allFinite(f));
  printf("PASS invalid measurement handling\n");
}

static void testLongRun()
{
  StaticDecoupledEstimator f;
  f.reset();
  for (int i = 0; i < 10000; ++i)
  {
    float t = i * 0.001f;
    float a = 0.2f * sinf(2.0f * PI_F * 0.5f * t);
    assert(f.predict(a, 0.001f));
    assert(f.updateGyro(0.2f * sinf(2.0f * PI_F * 0.3f * t)));
    if ((i % 10) == 0)
    {
      assert(f.updateWheelV(0.15f + 0.05f * sinf(2.0f * PI_F * 0.2f * t)));
      assert(f.updateWheelOmega(0.2f * sinf(2.0f * PI_F * 0.4f * t)));
    }
  }
  assert(allFinite(f));
  assert(f.healthy);
  printf("PASS 10000 mixed steps: finite state\n");
}

int main()
{
  testWrapAngle();
  testPredictKinematics();
  testAccelerometerTracking();
  testAccelerometerBias();
  testGyroAndWheelOmega();
  testReset();
  testInvalidMeasurements();
  testLongRun();
  printf("All static estimator host checks passed.\n");
  return 0;
}
