// c++ -std=c++14 -O2 -Isrc/src tests/motion_host/turn_rate_check.cpp -o /tmp/turn-rate-check
#include "turn_rate_calibration.h"
#include <cassert>
#include <climits>
#include <cstdio>

int main()
{
  assert(tuning::mainExperiment == tuning::MainExperiment::TurnRateCalibration);
  assert(tuning::duration(tuning::turnRateCalibration) == 40);
  assert(turnrate::command(3.999f).motorsOff);
  assert(!turnrate::command(4).motorsOff);
  assert(turnrate::command(4).reference == .2f);
  assert(turnrate::command(8).motorsOff);
  assert(turnrate::command(10).reference == -.2f);
  assert(turnrate::command(40).index == -1);
  assert(turnrate::command(NAN).index == -1);
  const unsigned capacity = (2291 * 216) / 224;
  assert(40000/tuning::logDivider(40,capacity)+1 < capacity);

  turnrate::ButtonLatch button;
  assert(button.update(true));
  for (int i=0; i<40000; ++i) assert(!button.update(true));
  assert(!button.update(false));
  assert(button.update(true));

  turnrate::Mean3 mean;
  const float offset[3] = {.1f,-.2f,.3f};
  const float invalid[3] = {NAN,0,0};
  assert(!mean.add(invalid,1000));
  for (int i=0; i<1000; ++i) assert(mean.add(offset,1000));
  assert(!mean.add(invalid,1000));
  assert(mean.count == 1000 && mean.value[1] == offset[1]);
  mean = {};
  for (int i=0; i<2000; ++i) mean.add(offset,2000);
  const float moved[3] = {10,20,30};
  assert(!mean.add(moved,2000)); // frozen before the motor sequence
  assert(mean.value[2] == offset[2]);

  turnrate::Angles angles;
  const float rates[3] = {1,0,-.5f};
  uint32_t timestamp = UINT_MAX-3000;
  const uint32_t first = timestamp;
  assert(angles.add(rates,timestamp));
  for (int i=0; i<8000; ++i)
  {
    timestamp += i%2 ? 1200 : 800;
    assert(angles.add(rates,timestamp));
  }
  assert(angles.value[0] > 6.2831853f);
  assert(std::fabs(angles.value[0]-(timestamp-first)*1e-6f) < .001f);
  assert(!angles.add(rates,timestamp));
  assert(!angles.add(rates,timestamp+20001));
  assert(!angles.add(invalid,timestamp+1000));

  turnrate::EncoderAngles encoder;
  encoder.seed(INT_MAX-2,INT_MAX-2);
  encoder.add(INT_MIN+2,INT_MIN+2,.001f,.14f);
  assert(std::fabs(encoder.distance-.005f) < 1e-6f && encoder.wheelAngle == 0);

  turnrate::Run run;
  unsigned offTicks=0;
  for (int i=0; i<=40000; ++i)
  {
    const float elapsed = i*.001f;
    const auto command = turnrate::command(elapsed);
    if (command.index != run.index) run.transition(command.index,elapsed);
    if (command.index < 0) continue;
    const float measured[3] = {-command.reference/2.4f,0,0};
    assert(run.gyro.add(measured,1000u+i*1000u));
    ++run.phases[command.index].ticks;
    if (command.motorsOff) ++offTicks;
  }
  assert(offTicks == 16000);
  for (const auto &phase : run.phases) assert(phase.recorded);
  assert(std::fabs(run.phases[1].gyroAngle[0]+.2f*4/2.4f) < .001f);
  assert(std::fabs(run.phases[3].gyroAngle[0]-.2f*4/2.4f) < .001f);
  // A repeat starts from new state, not from previous angles or summaries.
  run = {};
  assert(run.gyro.count == 0 && run.index == -1 && !run.phases[0].recorded);
  std::puts("PASS turn-rate sequence: calibration, timing, button latch, integrals, wrapping, summaries, reset");
}
