// c++ -std=c++14 -O2 -Isrc/src tests/motion_host/lecture10_check.cpp -o /tmp/lecture10-check
#include "trajectory.h"

#include <cassert>
#include <cmath>
#include <cstdio>

using namespace lecture10;

static void checkPivotRamps(float from, float target)
{
  const auto &p = rampedFullRoute;
  const float dt = 0.001f;
  const float end = pivotDuration(p);
  assert(p.pivotRise + p.pivotFall < end);
  assert(std::fabs(pivotReference(p, end / 2, from, target).yawRate) <= p.pivotSpeed);
  assert(pivotReference(p, p.pivotRise, from, target).yawRate == -p.pivotSpeed);
  float angle = from;
  auto previous = pivotReference(p, 0, from, target);
  assert(previous.yawRate == 0 && previous.heading == from);
  for (int tick = 1; (tick - 1) * dt < end; ++tick)
  {
    const auto ref = pivotReference(p, tick * dt, from, target);
    assert(ref.yawRate <= 0 && ref.yawRate >= -p.pivotSpeed);
    const float acceleration = (ref.yawRate - previous.yawRate) / dt;
    assert(acceleration >= -p.pivotSpeed / p.pivotRise - 0.001f);
    assert(acceleration <= p.pivotSpeed / p.pivotFall + 0.001f);
    angle += (previous.yawRate + ref.yawRate) * 0.5f * dt;
    assert(std::fabs(ref.heading - angle) < 0.0001f);
    previous = ref;
  }
  assert(previous.yawRate == 0 && previous.heading == target);
}

static void checkIdealRun(State &run, const Profile &profile)
{
  const double dt = 0.001;
  double distance = 0, heading = 0, yawRate = 0, x = 0, y = 0;
  Phase last = Rest;
  float holdStart = 0;
  bool crossedPi = false;
  bool reachedExitCruise = false;
  for (int tick = 0; tick <= 35000; ++tick)
  {
    const float t = tick * dt;
    const Reference ref = lecture10TrajectoryCommand(run, t, distance, heading, yawRate, profile);
    if (run.phase != last)
    {
      assert((last == Rest && run.phase == Entry) || (last == Entry && run.phase == Pivot) ||
             (last == Pivot && run.phase == Arc) ||
             (last == Arc && run.phase == (profile.driveExit ? ExitPivot : Hold)) ||
             (last == ExitPivot && run.phase == Exit) || (last == Exit && run.phase == Hold) ||
             (last == Hold && run.phase == Complete));
      if (last == Arc)
        assert(std::fabs(x - 1.00) < 0.003 && std::fabs(y - 0.30) < 0.003);
      if (run.phase == Hold) holdStart = t;
      last = run.phase;
    }
    if (run.phase == Complete)
    {
      assert(std::fabs(t - holdStart - 0.60) < 0.0011);
      assert(std::fabs(x - (profile.driveExit ? -0.50 : 1.00)) < 0.003);
      // This feed-forward-only integration carries the arc's last 1 mm of
      // distance tolerance into the 1.50 m exit as a small lateral error.
      assert(std::fabs(y - 0.30) < (profile.driveExit ? 0.005 : 0.003));
      assert(reachedExitCruise == profile.driveExit);
      assert(crossedPi && std::fabs(heading - (profile.driveExit ? 3.14159 : 4.40770)) < 0.004);
      std::printf("Ideal trial %.3f s, endpoint %.4f m forward, %.4f m left\n", t, x, y);
      return;
    }
    if (run.phase == Rest || run.phase == Hold)
      assert(ref.velocity == 0 && ref.yawRate == 0);
    if (run.phase == Hold)
      assert(std::fabs(ref.heading - (profile.driveExit ? 3.14159 : 4.40770)) < 0.0001);
    if (run.phase == Pivot || run.phase == ExitPivot)
      assert(ref.velocity == 0 && ref.yawRate <= 0);
    if (run.phase == Exit)
    {
      assert(ref.velocity > 0 && ref.velocity <= profile.straightSpeed);
      assert(ref.yawRate == 0 && std::fabs(ref.heading - 3.14159) < 0.0001);
      if (ref.velocity == profile.straightSpeed) reachedExitCruise = true;
    }
    if (run.phase == Arc)
    {
      assert(ref.velocity > 0 && ref.velocity <= profile.arcSpeed);
      assert(std::fabs(ref.yawRate - ref.velocity / 0.5) < 1e-6);
      assert(std::fabs(ref.heading - heading) < 0.002);
      if (ref.heading > 3.1416) crossedPi = true;
    }
    yawRate = ref.yawRate;
    const double ds = ref.velocity * dt;
    x += ds * std::cos(heading + yawRate * dt / 2);
    y += ds * std::sin(heading + yawRate * dt / 2);
    heading += yawRate * dt;
    distance += ds;
  }
  assert(false && "ideal trial did not finish");
}

int main()
{
  State run;
  checkIdealRun(run, arcTrial);
  run = {};  // same reset as a second accepted start in the firmware
  checkIdealRun(run, fullRoute);
  run = {};
  checkIdealRun(run, rampedFullRoute);
  checkPivotRamps(0, -pivotAngle);
  checkPivotRamps(arcAngle - pivotAngle, exitHeading);

  run = {};
  lecture10TrajectoryCommand(run, 1, 1, 0, 0);
  // The reference reaches the tangent even when the wheels never pivot.
  auto ref = lecture10TrajectoryCommand(run, 5, 1, 0, 0);
  assert(run.phase == Pivot && ref.velocity == 0 && ref.yawRate == 0);
  assert(std::fabs(ref.heading + 1.26610f) < 1e-5f);
  lecture10TrajectoryCommand(run, 6, 1, -pivotAngle, -0.4f);
  assert(run.phase == Pivot);
  lecture10TrajectoryCommand(run, 7, 1, -pivotAngle, 0);
  assert(run.phase == Arc);
  ref = lecture10TrajectoryCommand(run, 12, 1.25f, -pivotAngle + 0.4f, 0.4f);
  assert(std::fabs(ref.heading - (-pivotAngle + 0.50f)) < 1e-6f);
  // With distance frozen, elapsed time and heading alone cannot finish the arc.
  ref = lecture10TrajectoryCommand(run, 34.99f, 1.25f, arcAngle - pivotAngle, 0);
  assert(run.phase == Arc);
  ref = lecture10TrajectoryCommand(run, 35, 1.25f, arcAngle - pivotAngle, 0);
  assert(run.phase == TimedOut && ref.velocity == 0 && ref.yawRate == 0);

  run = {};
  ref = lecture10TrajectoryCommand(run, 34.99f, 0, 0, 0);
  assert(run.phase == Entry && ref.velocity == arcTrial.straightSpeed);
  ref = lecture10TrajectoryCommand(run, 35, 0, 0, 0);
  assert(run.phase == TimedOut && ref.velocity == 0 && ref.yawRate == 0);
  run = {};
  ref = lecture10TrajectoryCommand(run, 1, NAN, 0, 0);
  assert(run.phase == InvalidInput && ref.velocity == 0 && ref.yawRate == 0);

  // A delayed exit pivot must not spend any of the 1.50 m return leg.
  run = {};
  run.phase = ExitPivot;
  run.phaseStart = 20;
  ref = lecture10TrajectoryCommand(run, 23, 3.9f, 3.4f, 0, fullRoute);
  assert(run.phase == ExitPivot && ref.velocity == 0 && ref.yawRate == 0);
  ref = lecture10TrajectoryCommand(run, 24, 3.9f, exitHeading, -0.2f, fullRoute);
  assert(run.phase == ExitPivot && ref.velocity == 0);
  ref = lecture10TrajectoryCommand(run, 25, 4.0f, exitHeading, 0, fullRoute);
  assert(run.phase == Exit && ref.velocity == 0.30f);
  ref = lecture10TrajectoryCommand(run, 30, 5.45f, exitHeading, 0, fullRoute);
  assert(run.phase == Exit && ref.velocity > 0 && ref.velocity < 0.30f);
  ref = lecture10TrajectoryCommand(run, 31, 5.50f, exitHeading, 0, fullRoute);
  assert(run.phase == Hold && ref.velocity == 0);
  std::puts("Lecture 10 trajectory checks passed.");
}
