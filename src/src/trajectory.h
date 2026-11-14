#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <math.h>

struct TrajectoryConfig
{
  float initialRest = 0.2f;
  float forwardSpeed = 0.20f;      // m/s
  float forwardDuration = 4.0f;    // s
  float turnRate = 0.5235987756f;  // rad/s, positive CCW
  float turnDuration = 3.0f;       // s
  float finalRest = 0.4f;          // s

  float duration() const
  {
    return initialRest + forwardDuration + turnDuration + finalRest;
  }

  bool valid() const
  {
    return isfinite(initialRest) && initialRest >= 0.0f &&
           isfinite(forwardSpeed) && forwardSpeed > 0.0f &&
           isfinite(forwardDuration) && forwardDuration > 0.0f &&
           isfinite(turnRate) && fabsf(turnRate) > 0.0f &&
           isfinite(turnDuration) && turnDuration > 0.0f &&
           isfinite(finalRest) && finalRest >= 0.0f &&
           isfinite(duration());
  }
};

struct TrajectoryCommand
{
  float velocity;
  float yawRate;
  int phase;
};

inline TrajectoryCommand trajectoryCommand(const TrajectoryConfig &c, float elapsed)
{
  if (!c.valid() || !isfinite(elapsed) || elapsed < 0.0f)
    return {0.0f, 0.0f, 0};

  float end = c.initialRest;
  if (elapsed < end)
    return {0.0f, 0.0f, 10};

  end += c.forwardDuration;
  if (elapsed < end)
    return {c.forwardSpeed, 0.0f, 11};

  end += c.turnDuration;
  if (elapsed < end)
    return {0.0f, c.turnRate, 12};

  end += c.finalRest;
  if (elapsed < end)
    return {0.0f, 0.0f, 2};

  return {0.0f, 0.0f, 0};
}

#endif
