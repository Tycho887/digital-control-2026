#ifndef BASEBOT_TUNING_H
#define BASEBOT_TUNING_H

#include "pid.h"

namespace tuning
{
enum class MainExperiment { TurnRateCalibration, OriginalTrajectory };
constexpr MainExperiment mainExperiment = MainExperiment::OriginalTrajectory;
constexpr float sampleTime = 0.001f;
// Experiment 13 A/B trial: A uses experiment 11 trapezoids, B uses paired Simpson.
constexpr bool useSimpsonOdometry = false;

// ============================================================================
// Controller tuning
// ----------------------------------------------------------------------------
// Single location for every controller coefficient and limit. The CCDE form
// at Ts = 1 ms is
//   u[k] = b0*e[k] + b1*e[k-1] + b2*e[k-2] - a1*u[k-1] - a2*u[k-2]
// Kp/Ki values in comments use the Tustin PI conversion
//   b0 = Kp + Ki*Ts/2, b1 = -Kp + Ki*Ts/2, a1 = -1.
// ============================================================================

// Inner wheel-speed PI: motor-shaft rad/s error -> motor voltage (V).
// Left Kp = 0.05, Ki = 0.1; right Kp = 0.04, Ki = 0.1.
constexpr PID_Coefficients leftWheelCoeffs =
    {0.05005f, -0.04995f, 0.0f, -1.0f, 0.0f};
constexpr PID_Coefficients rightWheelCoeffs =
    {0.04005f, -0.03995f, 0.0f, -1.0f, 0.0f};

// Heading lead-lag: wrapped heading error (rad) -> yaw-rate correction
// (rad/s). Includes Kph = 2.5.
constexpr PID_Coefficients headingCoeffs =
    {4.996878901f, -4.990636704f, 0.0f, -0.997503121f, 0.0f};

// Heading-rate (yaw-rate) PI: rate error (rad/s) -> yaw-rate correction
// (rad/s). Kp = 2.5, Ki = 12.5.
constexpr PID_Coefficients yawRateCoeffs =
    {2.50625f, -2.49375f, 0.0f, -1.0f, 0.0f};

// Output limits. Lim_TU bounds the heading/yaw outputs (rad/s) and is also
// the turn limiter's upper rate bound (rad/s^2); Lim_TL is the lower rate
// bound.
constexpr float wheelVoltageLimit = 9.0f; // wheel PI output (V)
constexpr float Lim_TU = 2.5f;
constexpr float Lim_TL = -2.5f;

// Reference limiters. KpLV/KpLT are approach gains (1/s); Lim_V*/Lim_T*
// bound acceleration (m/s^2) and angular acceleration (rad/s^2).
constexpr float KpLV = 25.0f;
constexpr float KpLT = 25.0f;
constexpr float Lim_VU = 6.0f;
constexpr float Lim_VL = -6.0f;

// Encoder velocity alpha-beta filters (counts -> counts/s).
constexpr float leftVelAlpha = 0.5f;
constexpr float leftVelBeta = 0.1f;
constexpr float rightVelAlpha = 0.8f;
constexpr float rightVelBeta = 0.15f;

// MPU9250_asukiaaa already negates each hardware gyro axis. These choices
// refer to its reported axes, and must be checked with a measured rotation.
constexpr int yawAxis = 0;
constexpr float yawSign = -1.0f;
constexpr float yawScale = 1.0f;

struct Step
{
  float reference;
  float duration;
};

constexpr Step track[] = {
    {0, 1}, {0.05f, 2}, {0.10f, 2}, {0.15f, 2}, {0.20f, 2}, {0, 1},
    {-0.05f, 2}, {-0.10f, 2}, {-0.15f, 2}, {-0.20f, 2}, {0, 1}};
constexpr Step yaw[] = {
    {0, 2}, {0.2f, 3}, {0, 2}, {-0.2f, 3}, {0, 2},
    {0.5235987756f, 3}, {0, 2}, {-0.5235987756f, 3}, {0, 2}};

// One button press: stationary calibration, then independent wheel-speed
// steps with motors-off pauses. Heading/yaw PI corrections are not used.
constexpr Step turnRateCalibration[] = {
    {0, 4}, {0.2f, 4}, {0, 2}, {-0.2f, 4}, {0, 2},
    {0.4f, 4}, {0, 2}, {-0.4f, 4}, {0, 2},
    {0.6f, 4}, {0, 2}, {-0.6f, 4}, {0, 2}};

template <unsigned N>
constexpr float duration(const Step (&steps)[N])
{
  float total = 0;
  for (const Step &step : steps) total += step.duration;
  return total;
}

// Tustin PI conversion for RAM-only `cal gains` trials and host checks.
// u[k] = u[k-1] + b0*e[k] + b1*e[k-1].
constexpr PID_Coefficients pi(float kp, float ki)
{
  return {kp + ki * sampleTime / 2, -kp + ki * sampleTime / 2,
          0, -1, 0};
}

constexpr unsigned logDivider(float seconds, unsigned capacity)
{
  return static_cast<unsigned>(seconds / sampleTime / (capacity - 2)) + 1;
}
} // namespace tuning
#endif
