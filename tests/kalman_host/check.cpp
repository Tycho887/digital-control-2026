// Host-side checks for the 7-state differential-drive Kalman filter.
// Build and run from the project root:
//   c++ -std=c++14 -O2 -Isrc/src tests/kalman_host/check.cpp src/src/kalman.cpp -o /tmp/kalman-check
#include "kalman.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static const float PI_F = 3.14159265358979f;

static bool near(float a, float b, float tol)
{
  return fabsf(a - b) <= tol;
}

static void testMatrixMultiply2x2()
{
  const float A[4] = {1, 2, 3, 4};
  const float B[4] = {5, 6, 7, 8};
  float C[4];
  matrixMultiply(C, A, B, 2);
  assert(near(C[0], 19.0f, 1e-6f));
  assert(near(C[1], 22.0f, 1e-6f));
  assert(near(C[2], 43.0f, 1e-6f));
  assert(near(C[3], 50.0f, 1e-6f));
  printf("PASS matrixMultiply 2x2\n");
}

static void testMatrixMultiply3x3()
{
  const float A[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  const float B[9] = {9, 8, 7, 6, 5, 4, 3, 2, 1};
  const float expected[9] = {30, 24, 18, 84, 69, 54, 138, 114, 90};
  float C[9];
  matrixMultiply(C, A, B, 3);
  for (int i = 0; i < 9; ++i)
    assert(near(C[i], expected[i], 1e-5f));
  printf("PASS matrixMultiply 3x3\n");
}

static void testMatrixTranspose()
{
  const float A[4] = {1, 2, 3, 4};
  float AT[4];
  matrixTranspose(AT, A, 2);
  assert(near(AT[0], 1.0f, 1e-6f));
  assert(near(AT[1], 3.0f, 1e-6f));
  assert(near(AT[2], 2.0f, 1e-6f));
  assert(near(AT[3], 4.0f, 1e-6f));

  const float A3[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  float A3T[9];
  matrixTranspose(A3T, A3, 3);
  assert(near(A3T[0], 1.0f, 1e-6f));
  assert(near(A3T[1], 4.0f, 1e-6f));
  assert(near(A3T[2], 7.0f, 1e-6f));
  assert(near(A3T[3], 2.0f, 1e-6f));
  assert(near(A3T[4], 5.0f, 1e-6f));
  assert(near(A3T[5], 8.0f, 1e-6f));
  assert(near(A3T[6], 3.0f, 1e-6f));
  assert(near(A3T[7], 6.0f, 1e-6f));
  assert(near(A3T[8], 9.0f, 1e-6f));
  printf("PASS matrixTranspose 2x2 and 3x3\n");
}

static void testMatrixVectorMultiply()
{
  const float A[4] = {1, 2, 3, 4};
  const float v[2] = {1, 2};
  float y[2];
  matrixVectorMultiply(y, A, v, 2);
  assert(near(y[0], 5.0f, 1e-6f));
  assert(near(y[1], 11.0f, 1e-6f));

  const float A3[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  const float v3[3] = {1, 1, 1};
  float y3[3];
  matrixVectorMultiply(y3, A3, v3, 3);
  assert(near(y3[0], 6.0f, 1e-5f));
  assert(near(y3[1], 15.0f, 1e-5f));
  assert(near(y3[2], 24.0f, 1e-5f));
  printf("PASS matrixVectorMultiply 2x2 and 3x3\n");
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
  DiffDriveKalmanFilter f;
  f.reset();
  for (int i = 0; i < 1000; ++i)
    assert(f.predict(1.0f, 0.001f));
  assert(near(f.velocity(), 1.0f, 1e-3f));
  assert(near(f.distance(), 0.5f, 5e-3f));
  assert(near(f.x[DiffDriveKalmanFilter::BA], 0.0f, 1e-6f));
  assert(f.healthy);
  printf("PASS predict from rest: v=%.4f s=%.4f\n", f.velocity(), f.distance());

  assert(!f.predict(1.0f, 0.0f));
  assert(!f.predict(1.0f, 0.051f));
  assert(!f.predict(1.0f, -0.001f));
  assert(!f.predict(NAN, 0.001f));
  assert(!f.predict(INFINITY, 0.001f));
  assert(!f.predict(1.0f, NAN));
  printf("PASS predict input validation\n");
}

static void testGyroUpdate()
{
  DiffDriveKalmanFilter f;
  f.reset();
  // Isolated yaw-rate check: a constant omega + b_g reading is split between
  // the two states, so constrain b_g to its true value of zero here.
  f.P[DiffDriveKalmanFilter::BG * KF_N + DiffDriveKalmanFilter::BG] = 1e-8f;
  for (int i = 0; i < 50; ++i)
    f.updateGyro(0.5f);
  assert(near(f.yawRate(), 0.5f, 1e-3f));
  assert(near(f.x[DiffDriveKalmanFilter::BG], 0.0f, 1e-3f));
  assert(f.healthy);
  printf("PASS gyro update converges: omega=%.5f\n", f.yawRate());
}

static void testWheelUpdates()
{
  DiffDriveKalmanFilter f;
  f.reset();
  for (int i = 0; i < 200; ++i)
  {
    f.predict(0.0f, 0.001f);
    f.updateWheelV(0.3f, 0.01f);
    f.updateWheelOmega(0.5f, 0.01f);
  }
  assert(near(f.velocity(), 0.3f, 1e-3f));
  assert(near(f.yawRate(), 0.5f, 1e-3f));
  assert(f.healthy);
  printf("PASS wheel updates converge: v=%.5f omega=%.5f\n",
         f.velocity(), f.yawRate());
}

static void testMeasurementNoise()
{
  DiffDriveKalmanFilter f;
  // Gyro: (0.1 deg/s * pi/180)^2 from the MPU-9250 DLPF-92 Hz datasheet row.
  assert(near(f.rGyro(), 3.046174e-6f, 1e-12f));
  // Magnetometer: conservative 0.01 rad (1 sigma).
  assert(near(f.rMag(), 1.0e-4f, 1e-12f));
  // AS5X47 angle noise 0.068 deg, endpoint difference over a 10 ms window
  // (sqrt(2) * sigma_angle):
  //   R_v     = (sqrt(2) r sigma / (gear T))^2         = 3.033e-7 (m/s)^2
  //   R_omega = (sqrt(2) 2 r sigma / (b gear T))^2     = 6.190e-5 (rad/s)^2
  assert(near(f.rWheelV(0.01f), 3.033e-7f, 1e-9f));
  assert(near(f.rWheelOmega(0.01f), 6.190e-5f, 1e-7f));
  printf("PASS measurement variances: R_gyro=%.9g R_mag=%.9g "
         "R_wheel_v=%.9g R_wheel_omega=%.9g\n",
         f.rGyro(), f.rMag(), f.rWheelV(0.01f), f.rWheelOmega(0.01f));
}

static void testMagWrapTracking()
{
  DiffDriveKalmanFilter f;
  f.reset(3.0f);              // seed heading from the first mag sample
  float heading = 3.0f;
  for (int i = 0; i < 800; ++i)
  {
    heading += 0.001f;        // crosses +pi at i = 142
    f.predict(0.0f, 0.001f);
    f.updateMag(wrapAngle(heading));
    assert(!f.magRejected);
  }
  assert(heading > PI_F);
  assert(fabsf(wrapAngle(f.heading() - heading)) < 0.05f);
  assert(f.heading() > PI_F); // state must not jump to the negative branch
  assert(f.healthy);
  printf("PASS magnetometer wrap tracking: heading=%.5f truth=%.5f\n",
         f.heading(), heading);
}

static void testMagOutlierRejection()
{
  DiffDriveKalmanFilter f;
  f.reset(0.7f);
  for (int i = 0; i < 500; ++i)
  {
    f.predict(0.0f, 0.001f);
    f.updateMag(0.7f);
  }
  assert(!f.magRejected);
  float headingBefore = f.heading();

  f.updateMag(wrapAngle(0.7f + PI_F)); // 180 degree outlier
  assert(f.magRejected);
  assert(near(f.heading(), headingBefore, 1e-6f));

  f.updateMag(0.7f);                   // normal reading accepted again
  assert(!f.magRejected);
  assert(near(f.heading(), 0.7f, 0.02f));
  printf("PASS magnetometer outlier rejected, normal reading accepted\n");
}

static void testReset()
{
  DiffDriveKalmanFilter f;
  f.reset(0.7f);
  assert(near(f.heading(), 0.7f, 1e-6f));
  for (int i = 0; i < KF_N; ++i)
    if (i != DiffDriveKalmanFilter::THETA)
      assert(f.x[i] == 0.0f);
  assert(f.healthy);
  assert(!f.magRejected);

  assert(near(f.P[0 * KF_N + 0], 1e-4f, 1e-9f));
  assert(near(f.P[1 * KF_N + 1], 1e-2f, 1e-9f));
  assert(near(f.P[2 * KF_N + 2], 1e-2f, 1e-9f));
  assert(near(f.P[3 * KF_N + 3], 1e-2f, 1e-9f));
  assert(near(f.P[4 * KF_N + 4], 4e-2f, 1e-9f));
  assert(near(f.P[5 * KF_N + 5], 1e-4f, 1e-9f));
  assert(near(f.P[6 * KF_N + 6], 1e-2f, 1e-9f));
  printf("PASS reset heading and initial covariance\n");
}

static void testLongRun()
{
  DiffDriveKalmanFilter f;
  f.reset(0.0f);
  for (int i = 0; i < 10000; ++i)
  {
    float t = i * 0.001f;
    float a = 0.2f * sinf(2.0f * PI_F * 0.5f * t);
    assert(f.predict(a, 0.001f));
    f.updateGyro(0.2f * sinf(2.0f * PI_F * 0.3f * t));
    f.updateWheelV(0.15f + 0.05f * sinf(2.0f * PI_F * 0.2f * t), 0.01f);
    f.updateWheelOmega(0.2f * sinf(2.0f * PI_F * 0.4f * t), 0.01f);
    if ((i % 10) == 0)
      f.updateMag(wrapAngle(f.heading() + 0.01f * sinf(2.0f * PI_F * 0.1f * t)));
  }

  for (int i = 0; i < KF_N; ++i)
    assert(isfinite(f.x[i]));
  assert(f.healthy);

  for (int i = 0; i < KF_N; ++i)
  {
    assert(isfinite(f.P[i * KF_N + i]));
    assert(f.P[i * KF_N + i] > 0.0f);
    for (int j = i + 1; j < KF_N; ++j)
      assert(f.P[i * KF_N + j] == f.P[j * KF_N + i]);
  }
  printf("PASS 10000 mixed steps: finite state, symmetric P, positive diagonal\n");
}

int main()
{
  testMatrixMultiply2x2();
  testMatrixMultiply3x3();
  testMatrixTranspose();
  testMatrixVectorMultiply();
  testWrapAngle();
  testPredictKinematics();
  testGyroUpdate();
  testWheelUpdates();
  testMeasurementNoise();
  testMagWrapTracking();
  testMagOutlierRejection();
  testReset();
  testLongRun();
  printf("All Kalman host checks passed.\n");
  return 0;
}
