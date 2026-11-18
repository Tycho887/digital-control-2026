#ifndef BASEBOT_TUNING_H
#define BASEBOT_TUNING_H

#include "pid.h"

namespace tuning
{
enum class MainExperiment { TurnRateCalibration, OriginalTrajectory };
constexpr MainExperiment mainExperiment = MainExperiment::TurnRateCalibration;
constexpr float sampleTime = 0.001f;
constexpr float wheelVoltageLimit = 7.0f;
constexpr float leftKp = 0.05f;
constexpr float rightKp = 0.04f;
constexpr float wheelKi = 0.1f;
// Conservative trial gains; validate after measured yaw calibration.
constexpr float yawKp = 2.5f;
constexpr float yawKi = 12.5f;

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

// Tustin PI: u[k] = u[k-1] + b0*e[k] + b1*e[k-1].
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
