#ifndef TURN_RATE_CALIBRATION_H
#define TURN_RATE_CALIBRATION_H

#include "tuning.h"
#include <cmath>
#include <cstdint>

namespace turnrate
{
constexpr unsigned gyroCalibrationSamples = 1000;
constexpr unsigned accelCalibrationSamples = 2000;
constexpr unsigned phaseCount = sizeof(tuning::turnRateCalibration) /
                                sizeof(tuning::turnRateCalibration[0]);

struct Mean3
{
  unsigned count = 0;
  float value[3] = {};

  bool add(const float sample[3], unsigned target)
  {
    if (count >= target) return false;
    for (unsigned k = 0; k < 3; ++k)
      if (!std::isfinite(sample[k])) return false;
    ++count;
    for (unsigned k = 0; k < 3; ++k)
      value[k] += (sample[k] - value[k]) / count;
    return true;
  }
};

struct Command
{
  int index = -1;
  float reference = 0;
  bool motorsOff = true;
};

inline Command command(float elapsed)
{
  if (!std::isfinite(elapsed) || elapsed < 0) return {};
  float end = 0;
  for (unsigned i = 0; i < phaseCount; ++i)
  {
    end += tuning::turnRateCalibration[i].duration;
    if (elapsed < end)
      return {static_cast<int>(i), tuning::turnRateCalibration[i].reference,
              tuning::turnRateCalibration[i].reference == 0};
  }
  return {};
}

struct ButtonLatch
{
  bool held = false;
  bool update(bool pressed)
  {
    const bool rising = pressed && !held;
    held = pressed;
    return rising;
  }
};

struct Angles
{
  float value[3] = {};
  float previous[3] = {};
  uint32_t timestamp = 0;
  uint32_t count = 0;
  bool seeded = false;

  // Integrate every fresh sensor sample, before buffered-log decimation.
  // Timestamp subtraction also handles micros() wrapping.
  bool add(const float rate[3], uint32_t now)
  {
    for (unsigned k = 0; k < 3; ++k)
      if (!std::isfinite(rate[k])) return false;
    const uint32_t delta = now - timestamp;
    if (seeded && (delta == 0 || delta > 20000)) return false;
    if (seeded)
      for (unsigned k = 0; k < 3; ++k)
        value[k] += 0.5f * (previous[k] + rate[k]) * delta * 1e-6f;
    for (unsigned k = 0; k < 3; ++k) previous[k] = rate[k];
    timestamp = now;
    seeded = true;
    ++count;
    return true;
  }
};

struct EncoderAngles
{
  int32_t previous[2] = {};
  float wheelAngle = 0;
  float distance = 0;

  void seed(int32_t left, int32_t right) { previous[0] = left; previous[1] = right; }
  void add(int32_t left, int32_t right, float metresPerCount, float wheelBase)
  {
    const int32_t counts[2] = {left, right};
    float ds[2];
    for (unsigned k = 0; k < 2; ++k)
    {
      const int32_t delta = static_cast<int32_t>(static_cast<uint32_t>(counts[k]) -
                                               static_cast<uint32_t>(previous[k]));
      ds[k] = delta * metresPerCount;
      previous[k] = counts[k];
    }
    wheelAngle += (ds[1] - ds[0]) / wheelBase;
    distance += 0.5f * (ds[0] + ds[1]);
  }
};

struct PhaseSummary
{
  bool recorded = false;
  float start = 0, end = 0;
  float gyroAngle[3] = {};
  float wheelAngle = 0;
  uint32_t gyroSamples = 0, ticks = 0, saturatedTicks = 0;
};

struct Run
{
  Mean3 acceleration;
  Angles gyro;
  EncoderAngles encoder;
  PhaseSummary phases[phaseCount];
  int index = -1;
  float startAngle[3] = {}, startWheelAngle = 0;
  uint32_t startGyroCount = 0;
  bool measurementGap = false;
  const char *reason = "running";

  void transition(int next, float elapsed)
  {
    if (index >= 0)
    {
      PhaseSummary &summary = phases[index];
      summary.end = elapsed;
      summary.recorded = true;
      for (unsigned k = 0; k < 3; ++k)
        summary.gyroAngle[k] = gyro.value[k] - startAngle[k];
      summary.wheelAngle = encoder.wheelAngle - startWheelAngle;
      summary.gyroSamples = gyro.count - startGyroCount;
    }
    index = next;
    if (next >= 0)
    {
      phases[next].start = elapsed;
      for (unsigned k = 0; k < 3; ++k) startAngle[k] = gyro.value[k];
      startWheelAngle = encoder.wheelAngle;
      startGyroCount = gyro.count;
    }
  }
};
} // namespace turnrate
#endif
