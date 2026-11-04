#include "kalman.h"
#include <math.h>

namespace {
constexpr float KF_PI = 3.14159265358979f;
}

void matrixMultiply(float *C, const float *A, const float *B, int n)
{
  for (int i = 0; i < n; ++i)
  {
    for (int j = 0; j < n; ++j)
    {
      float sum = 0.0f;
      for (int k = 0; k < n; ++k)
        sum += A[i * n + k] * B[k * n + j];
      C[i * n + j] = sum;
    }
  }
}

void matrixVectorMultiply(float *y, const float *A, const float *x, int n)
{
  for (int i = 0; i < n; ++i)
  {
    float sum = 0.0f;
    for (int j = 0; j < n; ++j)
      sum += A[i * n + j] * x[j];
    y[i] = sum;
  }
}

void matrixTranspose(float *AT, const float *A, int n)
{
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j)
      AT[i * n + j] = A[j * n + i];
}

// Fold into (-pi, pi].
float wrapAngle(float a)
{
  a = fmodf(a, 2.0f * KF_PI);
  if (a <= -KF_PI)
    a += 2.0f * KF_PI;
  else if (a > KF_PI)
    a -= 2.0f * KF_PI;
  return a;
}

DiffDriveKalmanFilter::DiffDriveKalmanFilter()
{
  // The gyro and accelerometer offsets are calibrated at startup, so their
  // residual biases start small; otherwise the filter would absorb real
  // yaw rate into b_g before the gyro and mag updates can separate them.
  const float p0diag[KF_N] = {1e-4f, 1e-2f, 1e-2f, 1e-2f, 4e-2f, 1e-4f, 1e-2f};
  for (int i = 0; i < KF_N; ++i)
  {
    x[i] = 0.0f;
    B[i] = 0.0f;
  }
  for (int i = 0; i < KF_N * KF_N; ++i)
  {
    P[i] = 0.0f;
    F[i] = 0.0f;
    Q[i] = 0.0f;
    P0[i] = 0.0f;
  }
  for (int i = 0; i < KF_N; ++i)
    P0[i * KF_N + i] = p0diag[i];

  // MPU-9250 gyro total RMS noise 0.1 deg/s at DLPF 92 Hz:
  //   R_gyro = (0.1 * pi/180)^2 = 3.046174e-6 (rad/s)^2
  const float sigmaGyro = 0.1f * KF_PI / 180.0f;
  R_gyro = sigmaGyro * sigmaGyro;
  // Magnetometer heading has no datasheet noise figure; 0.01 rad (1 sigma)
  // is a conservative bound. The 0.15 uT/LSB quantisation floor is about
  // 2.5 mrad at a 17 uT horizontal field, so environmental noise dominates.
  R_mag = 1.0e-4f;

  reset();
}

void DiffDriveKalmanFilter::reset(float heading0)
{
  for (int i = 0; i < KF_N; ++i)
    x[i] = 0.0f;
  x[THETA] = heading0;
  for (int i = 0; i < KF_N * KF_N; ++i)
    P[i] = P0[i];
  healthy = true;
  magRejected = false;
}

bool DiffDriveKalmanFilter::predict(float aForward, float dt)
{
  if (!isfinite(aForward) || !isfinite(dt) || dt <= 0.0f || dt > 0.05f)
    return false;

  for (int i = 0; i < KF_N * KF_N; ++i)
    F[i] = 0.0f;
  for (int i = 0; i < KF_N; ++i)
    F[i * KF_N + i] = 1.0f;
  F[S * KF_N + V] = dt;
  F[THETA * KF_N + OMEGA] = dt;
  F[V * KF_N + BA] = -dt;

  for (int i = 0; i < KF_N; ++i)
    B[i] = 0.0f;
  B[V] = dt;

  // MPU-9250 accelerometer noise density 300 ug/sqrt(Hz) integrated over the
  // 94 Hz DLPF band: sigma_a = 300e-6 * 9.80665 * sqrt(94) = 0.028513 m/s^2.
  const float sA = 300e-6f * 9.80665f * sqrtf(94.0f);
  const float qv = 1e-2f, qomega = 1e-2f;
  const float qba = 1e-6f, qbg = 1e-8f, qbm = 1e-10f;
  const float dt2 = dt * dt;
  const float dt3 = dt2 * dt;
  const float dt4 = dt2 * dt2;

  for (int i = 0; i < KF_N * KF_N; ++i)
    Q[i] = 0.0f;
  Q[S * KF_N + S] = 0.25f * sA * sA * dt4;
  Q[S * KF_N + V] = 0.5f * sA * sA * dt3;
  Q[V * KF_N + S] = Q[S * KF_N + V];
  Q[V * KF_N + V] = sA * sA * dt2 + qv * dt;
  Q[OMEGA * KF_N + OMEGA] = qomega * dt;
  Q[BA * KF_N + BA] = qba * dt;
  Q[BG * KF_N + BG] = qbg * dt;
  Q[BM * KF_N + BM] = qbm * dt;

  float xNew[KF_N];
  matrixVectorMultiply(xNew, F, x, KF_N);
  for (int i = 0; i < KF_N; ++i)
    x[i] = xNew[i] + B[i] * aForward;

  float FP[KF_N * KF_N], FT[KF_N * KF_N], FPFt[KF_N * KF_N];
  matrixMultiply(FP, F, P, KF_N);
  matrixTranspose(FT, F, KF_N);
  matrixMultiply(FPFt, FP, FT, KF_N);
  for (int i = 0; i < KF_N * KF_N; ++i)
    P[i] = FPFt[i] + Q[i];

  symmetrize();
  healthy = healthy && finiteState();
  return true;
}

void DiffDriveKalmanFilter::updateScalar(const float *H, float z, float R)
{
  float PHt[KF_N];
  for (int i = 0; i < KF_N; ++i)
  {
    float sum = 0.0f;
    for (int j = 0; j < KF_N; ++j)
      sum += P[i * KF_N + j] * H[j];
    PHt[i] = sum;
  }

  float S = R;
  for (int i = 0; i < KF_N; ++i)
    S += H[i] * PHt[i];

  if (!isfinite(S) || S <= 0.0f || !isfinite(z))
    return;

  float innovation = z;
  for (int i = 0; i < KF_N; ++i)
    innovation -= H[i] * x[i];

  float K[KF_N];
  for (int i = 0; i < KF_N; ++i)
    K[i] = PHt[i] / S;

  for (int i = 0; i < KF_N; ++i)
    x[i] += K[i] * innovation;

  float HP[KF_N];
  for (int j = 0; j < KF_N; ++j)
  {
    float sum = 0.0f;
    for (int i = 0; i < KF_N; ++i)
      sum += H[i] * P[i * KF_N + j];
    HP[j] = sum;
  }
  for (int i = 0; i < KF_N; ++i)
    for (int j = 0; j < KF_N; ++j)
      P[i * KF_N + j] -= K[i] * HP[j];

  symmetrize();
  if (!finiteState())
    healthy = false;
}

void DiffDriveKalmanFilter::updateAngle(const float *H, float z, float R)
{
  float Hx = 0.0f;
  for (int i = 0; i < KF_N; ++i)
    Hx += H[i] * x[i];
  float zAdj = Hx + wrapAngle(z - Hx);
  updateScalar(H, zAdj, R);
}

// AS5X47 speed noise: the angle difference over one window has RMS noise
// sigma_angle, and the sensor sits on the motor shaft (wheel = angle / gear).
float DiffDriveKalmanFilter::rWheelV(float window) const
{
  const float sigma = KF_WHEEL_RADIUS * KF_AS5X47_SIGMA_ANGLE /
                      (KF_AS5X47_GEAR * window);
  return sigma * sigma;
}

float DiffDriveKalmanFilter::rWheelOmega(float window) const
{
  const float sigma = 2.0f * KF_WHEEL_RADIUS * KF_AS5X47_SIGMA_ANGLE /
                      (KF_WHEEL_BASE * KF_AS5X47_GEAR * window);
  return sigma * sigma;
}

void DiffDriveKalmanFilter::updateGyro(float z)
{
  const float H[KF_N] = {0, 0, 0, 1, 0, 1, 0};
  updateScalar(H, z, R_gyro);
}

void DiffDriveKalmanFilter::updateWheelV(float z, float window)
{
  const float H[KF_N] = {0, 0, 1, 0, 0, 0, 0};
  updateScalar(H, z, rWheelV(window));
}

void DiffDriveKalmanFilter::updateWheelOmega(float z, float window)
{
  const float H[KF_N] = {0, 0, 0, 1, 0, 0, 0};
  updateScalar(H, z, rWheelOmega(window));
}

void DiffDriveKalmanFilter::updateMag(float z)
{
  const float H[KF_N] = {0, 1, 0, 0, 0, 0, 1};

  float Hx = 0.0f;
  for (int i = 0; i < KF_N; ++i)
    Hx += H[i] * x[i];
  float y = wrapAngle(z - Hx);

  float PHt[KF_N];
  for (int i = 0; i < KF_N; ++i)
  {
    float sum = 0.0f;
    for (int j = 0; j < KF_N; ++j)
      sum += P[i * KF_N + j] * H[j];
    PHt[i] = sum;
  }
  float S = R_mag;
  for (int i = 0; i < KF_N; ++i)
    S += H[i] * PHt[i];

  if (y * y > 9.0f * S)
  {
    magRejected = true;
    return;
  }
  magRejected = false;
  updateAngle(H, z, R_mag);
}

void DiffDriveKalmanFilter::symmetrize()
{
  for (int i = 0; i < KF_N; ++i)
  {
    for (int j = i + 1; j < KF_N; ++j)
    {
      float avg = 0.5f * (P[i * KF_N + j] + P[j * KF_N + i]);
      P[i * KF_N + j] = avg;
      P[j * KF_N + i] = avg;
    }
  }
}

bool DiffDriveKalmanFilter::finiteState() const
{
  for (int i = 0; i < KF_N; ++i)
    if (!isfinite(x[i]))
      return false;
  return true;
}

DiffDriveKalmanFilter kf;
