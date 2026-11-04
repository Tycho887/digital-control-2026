#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>

constexpr int KF_N = 7;

// Robot geometry, used to turn AS5X47 angle differences into wheel speeds.
constexpr float KF_WHEEL_RADIUS = 0.0315f;  // driven wheel radius (m)
constexpr float KF_WHEEL_BASE = 0.14f;      // track width between wheels (m)
constexpr float KF_AS5X47_GEAR = 9.6f;      // motor shaft : wheel reduction

// AS5047P / AS5147 RMS output angle noise, 1 sigma with NOISESET=0: 0.068 deg.
constexpr float KF_AS5X47_SIGMA_ANGLE = 0.068f * 3.14159265358979f / 180.0f;

// Default angle-differencing window for the AS5X47 wheel speed updates (s).
constexpr float KF_WHEEL_WINDOW = 0.010f;

void matrixMultiply(float *C, const float *A, const float *B, int n);
void matrixVectorMultiply(float *y, const float *A, const float *x, int n);
void matrixTranspose(float *AT, const float *A, int n);
float wrapAngle(float a);

/**
 * 7-state differential-drive filter.
 * State: [s, theta, v, omega, b_a, b_g, b_m]
 * Matrices are row-major and sized KF_N * KF_N.
 * No Arduino headers, so the module also builds on the host.
 */
class DiffDriveKalmanFilter
{
public:
  enum StateIndex { S = 0, THETA = 1, V = 2, OMEGA = 3, BA = 4, BG = 5, BM = 6 };

  float x[KF_N];
  float P[KF_N * KF_N];
  bool healthy = true;
  bool magRejected = false;

  DiffDriveKalmanFilter();
  void reset(float heading0 = 0.0f);
  bool predict(float aForward, float dt);
  void updateGyro(float z);
  void updateWheelV(float z, float window = KF_WHEEL_WINDOW);
  void updateWheelOmega(float z, float window = KF_WHEEL_WINDOW);
  void updateMag(float z);

  // Measurement variances actually used by the updates.
  float rGyro() const { return R_gyro; }
  float rMag() const { return R_mag; }
  float rWheelV(float window) const;
  float rWheelOmega(float window) const;

  float distance() const { return x[S]; }
  float heading() const { return x[THETA]; }
  float velocity() const { return x[V]; }
  float yawRate() const { return x[OMEGA]; }

private:
  float F[KF_N * KF_N];
  float Q[KF_N * KF_N];
  float B[KF_N];
  float R_gyro, R_mag;
  float P0[KF_N * KF_N];
  void updateScalar(const float *H, float z, float R);
  void updateAngle(const float *H, float z, float R);
  void symmetrize();
  bool finiteState() const;
};

extern DiffDriveKalmanFilter kf;

#endif
