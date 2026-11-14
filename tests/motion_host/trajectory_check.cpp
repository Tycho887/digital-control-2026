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
  assert(near(c.duration(), 7.6f, 1e-6f));

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

  float distance = 0.0f;
  float heading = 0.0f;
  for (float t = 0.0f; t < c.duration(); t += 0.001f)
  {
    TrajectoryCommand command = trajectoryCommand(c, t);
    distance += command.velocity * 0.001f;
    heading += command.yawRate * 0.001f;
  }
  assert(near(distance, 0.8f, 0.001f));
  assert(near(heading, 1.5707963268f, 0.001f));
  printf("PASS trajectory phases: distance=%.4f m heading=%.5f rad\n",
         distance, heading);

  c.turnRate = -c.turnRate;
  assert(trajectoryCommand(c, turnStart).yawRate < 0.0f);
  c.turnRate = 0.0f;
  assert(!c.valid());
  assert(trajectoryCommand(c, turnStart).phase == 0);

  c = TrajectoryConfig{};
  c.forwardSpeed = 0.0f;
  assert(!c.valid());
  c = TrajectoryConfig{};
  c.forwardDuration = -1.0f;
  assert(!c.valid());
  c = TrajectoryConfig{};
  c.finalRest = NAN;
  assert(!c.valid());
  assert(trajectoryCommand(c, NAN).phase == 0);
  assert(trajectoryCommand(c, -0.001f).phase == 0);

  printf("All trajectory host checks passed.\n");
  return 0;
}
