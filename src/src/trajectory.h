#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <math.h>

struct TrajectoryConfig
{
  float initialRest = 0.2f;
  float forwardSpeed = 0.5f;      // mean m/s over the straight (2 m in 4 s)
  float forwardDuration = 4.0f;   // s
  float turnRate = -0.78539816f;  // mean rad/s over the turn (clockwise 180 deg in 4 s)
  float turnDuration = 4.0f;      // s
  float finalRest = 2.5f;         // s holding the starting heading after the final turn
  float turnRampTime = 0.5f;      // s at each end of the turn; 0 restores the step
  float forwardBrakeTime = 0.6f;  // s of braking at the end of the straight; 0 restores the step
  float turnSettle = 2.5f;        // s to observe the settled heading before the return leg
  float preTurnSettle = 0.5f;     // s at zero speed before each turn

  constexpr float duration() const
  {
    return initialRest + forwardDuration + preTurnSettle + turnDuration +
           turnSettle + forwardDuration + preTurnSettle + turnDuration + finalRest;
  }

  bool valid() const
  {
    return isfinite(initialRest) && initialRest >= 0.0f &&
           isfinite(forwardSpeed) && forwardSpeed > 0.0f &&
           isfinite(forwardDuration) && forwardDuration > 0.0f &&
           isfinite(forwardBrakeTime) && forwardBrakeTime >= 0.0f &&
           isfinite(turnRate) && fabsf(turnRate) > 0.0f &&
           isfinite(turnDuration) && turnDuration > 0.0f &&
           isfinite(turnRampTime) && turnRampTime >= 0.0f &&
           isfinite(turnSettle) && turnSettle >= 0.0f &&
           isfinite(preTurnSettle) && preTurnSettle >= 0.0f &&
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

inline float straightLegSpeed(const TrajectoryConfig &c, float remaining)
{
  const float brake = fminf(c.forwardBrakeTime, c.forwardDuration);
  if (brake == 0.0f)
    return c.forwardSpeed;

  // The cruise and linear brake cover cruiseSpeed * (forwardDuration - brake / 2).
  const float cruiseSpeed = c.forwardSpeed / (1.0f - 0.5f * brake / c.forwardDuration);
  return cruiseSpeed * fminf(1.0f, remaining / brake);
}

inline float turnProfileRate(const TrajectoryConfig &c, float timeToEdge)
{
  const float ramp = fminf(c.turnRampTime, 0.5f * c.turnDuration);
  if (ramp == 0.0f) return c.turnRate;
  // Two linear ramps and a plateau have area peak * (turnDuration - ramp).
  const float peak = c.turnRate / (1.0f - ramp / c.turnDuration);
  return peak * fminf(1.0f, timeToEdge / ramp);
}

inline TrajectoryCommand trajectoryCommand(const TrajectoryConfig &c, float elapsed)
{
  if (!c.valid() || !isfinite(elapsed) || elapsed < 0.0f)
    return {0.0f, 0.0f, 0};

  float end = c.initialRest;
  if (elapsed < end)
    return {0.0f, 0.0f, 10};

  end += c.forwardDuration;
  if (elapsed < end)
    return {straightLegSpeed(c, end - elapsed), 0.0f, 11};

  end += c.preTurnSettle;
  if (elapsed < end)
    return {0.0f, 0.0f, 15};

  const float turnStart = end;
  end += c.turnDuration;
  if (elapsed < end)
    return {0.0f, turnProfileRate(c, fminf(elapsed - turnStart, end - elapsed)), 12};

  end += c.turnSettle;
  if (elapsed < end)
    return {0.0f, 0.0f, 13};

  end += c.forwardDuration;
  if (elapsed < end)
    return {straightLegSpeed(c, end - elapsed), 0.0f, 14};

  end += c.preTurnSettle;
  if (elapsed < end)
    return {0.0f, 0.0f, 16};

  const float finalTurnStart = end;
  end += c.turnDuration;
  if (elapsed < end)
    return {0.0f, -turnProfileRate(c, fminf(elapsed - finalTurnStart, end - elapsed)), 17};

  end += c.finalRest;
  if (elapsed < end)
    return {0.0f, 0.0f, 2};

  return {0.0f, 0.0f, 0};
}

#endif
