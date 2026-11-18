#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <math.h>

struct TrajectoryConfig
{
  float initialRest = 0.2f;
  float forwardSpeed = 0.5f;      // m/s
  float forwardDuration = 4.0f;   // s
  float turnRate = 0.78539816f;   // mean rad/s over the turn (180 deg in 4 s)
  float turnDuration = 4.0f;      // s
  float finalRest = 0.6f;         // s
  float turnRampTime = 0.5f;      // s at each end of the turn; 0 restores the step

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
           isfinite(turnRampTime) && turnRampTime >= 0.0f &&
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

  const float turnStart = end;
  end += c.turnDuration;
  if (elapsed < end)
  {
    const float ramp = fminf(c.turnRampTime, 0.5f * c.turnDuration);
    if (ramp == 0.0f)
      return {0.0f, c.turnRate, 12};

    // Two linear ramps and a plateau have area peak * (turnDuration - ramp).
    const float peak = c.turnRate / (1.0f - ramp / c.turnDuration);
    const float timeToEdge = fminf(elapsed - turnStart, end - elapsed);
    return {0.0f, peak * fminf(1.0f, timeToEdge / ramp), 12};
  }

  end += c.finalRest;
  if (elapsed < end)
    return {0.0f, 0.0f, 2};

  return {0.0f, 0.0f, 0};
}

#endif
