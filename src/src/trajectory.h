#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include <math.h>

struct TrajectoryConfig
{
  float speed;           
  float radius;          
  float entryDistance;  
  float arcDistance;     
  float exitDistance;    
  float initialDelay;    
  float turnRate() const { return speed / radius; }  
  float totalDistance() const { return entryDistance + arcDistance + exitDistance; }
};

struct TrajectoryCommand
{
  float velocity;
  float yawRate;
  int phase;
};

inline TrajectoryCommand trajectoryCommand(const TrajectoryConfig &c, float distance, float time_elapsed)
{
  if (!isfinite(distance) || distance < 0.0f)
    return {0.0f, 0.0f, 0};

  if (time_elapsed < c.initialDelay)
    return {0.0f, 0.0f, 10};

  if (distance < c.entryDistance)
    return {c.speed, 0.0f, 11};

  if (distance < c.entryDistance + c.arcDistance)
    return {c.speed, c.turnRate(), 12};

  if (distance < c.totalDistance())
    return {c.speed, 0.0f, 13};

  return {0.0f, 0.0f, 0};
}

#endif