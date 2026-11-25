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

namespace lecture10
{
constexpr float entryDistance = 1.00f;  // m
constexpr float exitDistance = 1.50f;   // m, parallel to the entry in the -x direction
constexpr float radius = 0.50f;         // m, marking centreline
constexpr float gap = 0.30f;            // m between the parallel straights
constexpr float exitHeading = 3.14159265f;              // -x, unwrapped
const float pivotAngle = acosf(gap / (2.0f * radius));  // clockwise magnitude
const float arcAngle = exitHeading + 2.0f * pivotAngle; // long CCW sweep
const float arcDistance = radius * arcAngle;

struct Profile
{
  float straightSpeed;  // m/s
  float arcSpeed;       // m/s
  bool driveExit;
  float pivotSpeed;     // rad/s, clockwise magnitude
  float pivotRise;      // s, linear acceleration to peak rate
  float pivotFall;      // s, linear deceleration to rest
};
constexpr Profile arcTrial{0.20f, 0.20f, false, 0.5236f, 0.0f, 0.0f};
constexpr Profile fullRoute{0.30f, 0.25f, true, 0.5236f, 0.0f, 0.0f};
constexpr Profile rampedFullRoute{0.30f, 0.25f, true, 0.85f, 0.65f, 1.40f};

constexpr float brakingAccel = 0.40f;   // m/s^2
constexpr float distanceTolerance = 0.001f; // m
constexpr float pivotTolerance = 0.01745f;  // rad (1 degree)
constexpr float settledRate = 0.08f;        // rad/s before leaving either pivot
constexpr float initialRest = 0.20f;        // s
constexpr float stopCapture = 0.60f;        // s
constexpr float deadline = 35.0f;           // s from start, including stop capture

// Positive IDs appear in the standard log's state column.
enum Phase { InvalidInput = -2, TimedOut = -1, Complete = 0, Hold = 2,
             Rest = 10, Entry = 20, Pivot = 21, Arc = 22, ExitPivot = 23, Exit = 24 };

struct State
{
  Phase phase = Rest;
  float phaseStart = 0.0f;
  float legStartDistance = 0.0f;
};

struct Reference
{
  float velocity;
  float yawRate;
  float heading;  // unwrapped radians relative to the starting direction
};

inline float approachSpeed(float remaining, float speedLimit)
{
  return fminf(speedLimit, sqrtf(2.0f * brakingAccel * remaining));
}

// The heading reference is the integral of the commanded clockwise rate.
// A pair of zero ramp times gives the constant-rate pivot used in cycles 1 and 2.
inline float pivotDuration(const Profile &p)
{
  return pivotAngle / p.pivotSpeed + 0.5f * (p.pivotRise + p.pivotFall);
}

inline Reference pivotReference(const Profile &p, float t, float from, float target)
{
  if (t < p.pivotRise)
  {
    const float rate = p.pivotSpeed * t / p.pivotRise;
    return {0.0f, -rate, from - 0.5f * rate * t};
  }
  const float remaining = pivotDuration(p) - t;
  if (remaining <= 0.0f) return {0.0f, 0.0f, target};
  if (remaining < p.pivotFall)
  {
    const float rate = p.pivotSpeed * remaining / p.pivotFall;
    return {0.0f, -rate, target + 0.5f * rate * remaining};
  }
  return {0.0f, -p.pivotSpeed, from - p.pivotSpeed * (t - 0.5f * p.pivotRise)};
}

// distance and heading are signed odometry relative to start(); elapsed is wall time.
inline Reference lecture10TrajectoryCommand(State &run, float elapsed, float distance,
                                           float heading, float measuredYawRate,
                                           const Profile &profile = arcTrial)
{
  if (run.phase <= Complete) return {};
  if (!isfinite(elapsed) || elapsed < 0.0f || !isfinite(distance) ||
      !isfinite(heading) || !isfinite(measuredYawRate))
  {
    run.phase = InvalidInput;
    return {};
  }
  if (elapsed >= deadline)
  {
    run.phase = TimedOut;
    return {};
  }

  if (run.phase == Rest)
  {
    if (elapsed < initialRest) return {};
    run.phase = Entry;
  }
  if (run.phase == Entry)
  {
    const float remaining = entryDistance - distance;
    if (remaining > distanceTolerance)
      return {approachSpeed(remaining, profile.straightSpeed), 0.0f, 0.0f};
    run.phase = Pivot;
    run.phaseStart = elapsed;
  }
  if (run.phase == Pivot || run.phase == ExitPivot)
  {
    const bool atExit = run.phase == ExitPivot;
    const float target = atExit ? exitHeading : -pivotAngle;
    const float from = atExit ? arcAngle - pivotAngle : 0.0f;
    const float t = elapsed - run.phaseStart;
    if (t < pivotDuration(profile)) return pivotReference(profile, t, from, target);
    if (fabsf(heading - target) > pivotTolerance || fabsf(measuredYawRate) > settledRate)
      return {0.0f, 0.0f, target};
    run.phase = atExit ? Exit : Arc;
    run.legStartDistance = distance;
  }
  if (run.phase == Arc)
  {
    const float travelled = fmaxf(0.0f, distance - run.legStartDistance);
    const float remaining = arcDistance - travelled;
    if (remaining > distanceTolerance)
    {
      const float speed = approachSpeed(remaining, profile.arcSpeed);
      return {speed, speed / radius, -pivotAngle + travelled / radius};
    }
    run.phase = profile.driveExit ? ExitPivot : Hold;
    run.phaseStart = elapsed;
    return {0.0f, 0.0f, arcAngle - pivotAngle};
  }
  if (run.phase == Exit)
  {
    const float remaining = exitDistance - (distance - run.legStartDistance);
    if (remaining > distanceTolerance)
      return {approachSpeed(remaining, profile.straightSpeed), 0.0f, exitHeading};
    run.phase = Hold;
    run.phaseStart = elapsed;
  }
  if (elapsed - run.phaseStart >= stopCapture) run.phase = Complete;
  return {0.0f, 0.0f, profile.driveExit ? exitHeading : arcAngle - pivotAngle};
}
} // namespace lecture10

#endif
