// Build and run from the project root:
//   c++ -std=c++14 -O2 -Isrc/src tests/motion_host/trajectory_check.cpp \
//       -o /tmp/trajectory-check && /tmp/trajectory-check
#include "trajectory.h"

#include <cassert>
#include <cmath>
#include <cstdio>

static bool near(float a, float b, float tolerance)
{
  return std::fabs(a - b) <= tolerance;
}

int main()
{
  TrajectoryConfig c;
  assert(c.valid());
  assert(near(c.duration(), 8.8f, 1e-6f));

  const float turnStart = c.initialRest + c.forwardDuration;
  const float restStart = turnStart + c.turnDuration;
  assert(trajectoryCommand(c, 0.0f).phase == 10);
  assert(trajectoryCommand(c, c.initialRest - 0.001f).phase == 10);
  assert(trajectoryCommand(c, c.initialRest).phase == 11);
  assert(trajectoryCommand(c, turnStart - 0.001f).phase == 11);
  assert(trajectoryCommand(c, turnStart).phase == 12);
  assert(trajectoryCommand(c, restStart - 0.001f).phase == 12);
  assert(trajectoryCommand(c, restStart).phase == 2);
  assert(trajectoryCommand(c, c.duration() - 0.001f).phase == 2);
  assert(trajectoryCommand(c, c.duration()).phase == 0);

  // Experiment 4: a 2 m straight, with the final 0.60 s reserved for braking.
  const float cruiseSpeed = 2.0f / 3.7f;
  assert(near(trajectoryCommand(c, c.initialRest).velocity, cruiseSpeed, 1e-6f));
  assert(near(trajectoryCommand(c, 3.60f).velocity, cruiseSpeed, 1e-6f));
  assert(near(trajectoryCommand(c, 3.90f).velocity, cruiseSpeed / 2, 1e-6f));
  assert(near(trajectoryCommand(c, turnStart - 0.001f).velocity, cruiseSpeed / 600, 1e-6f));
  assert(trajectoryCommand(c, turnStart).velocity == 0.0f);

  float distance = 0.0f;
  float heading = 0.0f;
  for (float t = 0.0f; t < c.duration(); t += 0.001f)
  {
    TrajectoryCommand command = trajectoryCommand(c, t);
    distance += command.velocity * 0.001f;
    heading += command.yawRate * 0.001f;
  }
  assert(near(distance, 2.0f, 0.001f));
  assert(near(heading, 3.14159265f, 0.001f));
  printf("PASS trajectory phases: distance=%.4f m heading=%.5f rad\n",
         distance, heading);

  c.turnRampTime = 0.5f;
  const float peak = 3.14159265f / 3.5f;
  assert(near(trajectoryCommand(c, turnStart).yawRate, 0.0f, 1e-6f));
  assert(near(trajectoryCommand(c, turnStart + 0.25f).yawRate, peak / 2, 1e-6f));
  assert(near(trajectoryCommand(c, turnStart + 0.5f).yawRate, peak, 1e-6f));
  assert(near(trajectoryCommand(c, restStart - 0.5f).yawRate, peak, 1e-6f));
  assert(near(trajectoryCommand(c, restStart - 0.25f).yawRate, peak / 2, 1e-6f));
  assert(trajectoryCommand(c, restStart).yawRate == 0.0f);

  c.turnRate = -c.turnRate;
  assert(near(trajectoryCommand(c, turnStart + 0.25f).yawRate, -peak / 2, 1e-6f));
  c.turnDuration = 0.5f; // The ramps meet at the midpoint of a short turn.
  assert(near(trajectoryCommand(c, turnStart + 0.25f).yawRate, 2 * c.turnRate, 1e-6f));
  c.turnRampTime = 0.0f;
  assert(trajectoryCommand(c, turnStart).yawRate == c.turnRate);
  assert(trajectoryCommand(c, turnStart + 0.499f).yawRate == c.turnRate);
  c.turnRate = 0.0f;
  assert(!c.valid());
  assert(trajectoryCommand(c, turnStart).phase == 0);

  c = TrajectoryConfig{};
  c.forwardDuration = 0.4f; // Braking can occupy the entire straight.
  assert(near(trajectoryCommand(c, c.initialRest).velocity, 2 * c.forwardSpeed, 1e-6f));
  assert(near(trajectoryCommand(c, c.initialRest + 0.2f).velocity, c.forwardSpeed, 1e-6f));
  c.forwardBrakeTime = 0.0f;
  assert(trajectoryCommand(c, c.initialRest + 0.399f).velocity == c.forwardSpeed);
  c.forwardBrakeTime = -0.1f;
  assert(!c.valid());
  c.forwardBrakeTime = NAN;
  assert(!c.valid());

  c = TrajectoryConfig{};
  c.forwardSpeed = 0.0f;
  assert(!c.valid());
  c = TrajectoryConfig{};
  c.forwardDuration = -1.0f;
  assert(!c.valid());
  c = TrajectoryConfig{};
  c.turnRampTime = -0.5f;
  assert(!c.valid());
  c.turnRampTime = NAN;
  assert(!c.valid());
  c = TrajectoryConfig{};
  c.finalRest = NAN;
  assert(!c.valid());
  assert(trajectoryCommand(c, NAN).phase == 0);
  assert(trajectoryCommand(c, -0.001f).phase == 0);

  printf("All trajectory host checks passed.\n");
  return 0;
}
