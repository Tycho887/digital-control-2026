// c++ -std=c++14 -O2 -Isrc/src tests/motion_host/tuning_check.cpp -o /tmp/tuning-check
#include "tuning.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>

int main()
{
  assert(tuning::wheelVoltageLimit == 7.0f);
  assert(tuning::duration(tuning::track) == 19.0f);
  assert(tuning::duration(tuning::yaw) == 22.0f);
  // Actual shared-buffer capacity and sample counts must cover complete runs.
  const unsigned capacity = 495000 / 224;
  for (float duration : {19.0f, 22.0f})
  {
    const unsigned divider = tuning::logDivider(duration, capacity);
    assert(std::ceil(duration / tuning::sampleTime / divider) + 1 < capacity);
  }
  for (const auto &step : tuning::track)
    assert(std::fabs(step.reference) <= 0.20f && step.duration >= 1.0f);
  for (const auto &step : tuning::yaw)
    assert(std::fabs(step.reference) <= 0.524f && step.duration >= 2.0f);

  PIDController yaw(tuning::pi(2.5f, 12.5f), 1000);
  // Ten seconds of unit error: trapezoidal integral plus proportional term.
  for (int i = 0; i < 10000; ++i) yaw.ComputeCCDE(1);
  assert(std::fabs(yaw.lastOutput() - (2.5f + 12.5f * 9.9995f)) < 0.04f);
  yaw.setCoefficients(tuning::pi(1, 0));
  assert(yaw.lastOutput() == 0 && yaw.lastError() == 0);
  assert(yaw.ComputeCCDE(-1) == -1);
  std::puts("PASS tuning: PI conversion/reset, signed schedules, capture coverage");
}
