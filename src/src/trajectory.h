#pragma once
#include <math.h>

// Amounts are commanded travel and heading, implemented by time at fixed speed.
// This deliberately does not close a position loop around the diagnostic filter.
struct TrajectoryConfig {
  float firstDistance = 0.5f; // m, forward
  float turnAngle = 1.57079632679f; // rad; positive counterclockwise
  float secondDistance = 0.5f; // m, forward
  float speed = 0.15f; // m/s
  float turnRate = 0.50f; // rad/s magnitude
  float initialRest = 4.0f, betweenRest = 0.5f, finalRest = 1.0f;
  float duration() const {
    return initialRest+firstDistance/speed+betweenRest+
      fabsf(turnAngle)/turnRate+betweenRest+secondDistance/speed+finalRest;
  }
  bool valid() const {
    return isfinite(firstDistance)&&firstDistance>0 &&
      isfinite(secondDistance)&&secondDistance>0 &&
      isfinite(turnAngle)&&fabsf(turnAngle)>0 &&
      isfinite(speed)&&speed>0 && isfinite(turnRate)&&turnRate>0 &&
      isfinite(initialRest)&&initialRest>=0 &&
      isfinite(betweenRest)&&betweenRest>=0 && isfinite(finalRest)&&finalRest>0 &&
      isfinite(duration());
  }
};
struct TrajectoryCommand {
  float velocity, yawRate;
  int phase; // 10 baseline,11 forward,12 rest,13 turn,14 rest,15 forward,16 final rest,0 complete
};
inline TrajectoryCommand trajectoryCommand(const TrajectoryConfig& c, float elapsed) {
  if(!c.valid() || !isfinite(elapsed) || elapsed<0) return {0,0,0};
  float end=c.initialRest;
  if(elapsed<end) return {0,0,10};
  end+=c.firstDistance/c.speed;
  if(elapsed<end) return {c.speed,0,11};
  end+=c.betweenRest;
  if(elapsed<end) return {0,0,12};
  end+=fabsf(c.turnAngle)/c.turnRate;
  if(elapsed<end) return {0,c.turnAngle>0?c.turnRate:-c.turnRate,13};
  end+=c.betweenRest;
  if(elapsed<end) return {0,0,14};
  end+=c.secondDistance/c.speed;
  if(elapsed<end) return {c.speed,0,15};
  end+=c.finalRest;
  if(elapsed<end) return {0,0,16};
  return {0,0,0};
}
