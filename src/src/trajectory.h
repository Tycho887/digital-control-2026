#ifndef TRAJECTORY_H
#define TRAJECTORY_

struct TrajectoryConfig
{
  float speed;
  float radius;
  float entryDistance;
  float arcDistance;     // arc angle = arcDistance / radius
  float exitDistance;
  float initialDelay;
  float cornerAngle = 1.266f;   // rad
  float turnSpeed = 0.5236f;    // rad/s magnitude of the in-place turns
  float turnRate() const { return speed / radius; }
  // float totalDistance() const { return entryDistance + arcDistance + exitDistance; }
};

struct TrajectoryCommand
{
  float velocity;
  float yawRate;
  int phase;
};

struct TrajectoryState
{
  int stage = 0;
  bool seeded = false;
  float d0 = 0, th = 0, th0 = 0, prev = 0;
};

inline TrajectoryCommand trajectoryCommand(const TrajectoryConfig &c, TrajectoryState &st, float distance, float heading, float time_elapsed)
{
  if (!isfinite(distance) || !isfinite(heading) || !isfinite(time_elapsed))
    return {0.0f, 0.0f, -1};

  // unwrap heading
  if (!st.seeded) { st.prev = heading; st.seeded = true; }
  float dh = fmodf(heading - st.prev + 3.14159265f + 6.2831853f, 6.2831853f) - 3.14159265f;
  st.th += dh;
  st.prev = heading;

  if (time_elapsed < c.initialDelay)
  {
    st.d0 = distance; st.th0 = st.th;      
    return {0.0f, 0.0f, 10};
  }

  for (int guard = 0; guard < 6; ++guard)  
  {
    const float d = distance - st.d0;      
    const float a = st.th - st.th0;       
    switch (st.stage)
    {
      case 0: if (d < c.entryDistance)            return {c.speed, 0.0f, 11}; break;
      case 1: if (-a < c.cornerAngle)             return {0.0f, -c.turnSpeed, 14}; break;
      case 2: if (a < c.arcDistance / c.radius)   return {c.speed, c.turnRate(), 12}; break;
      case 3: if (-a < c.cornerAngle)             return {0.0f, -c.turnSpeed, 15}; break;
      case 4: if (d < c.exitDistance)             return {c.speed, 0.0f, 13}; break;
      default: return {0.0f, 0.0f, 0};
    }
    st.stage++; st.d0 = distance; st.th0 = st.th;
  }
  return {0.0f, 0.0f, 0};
}

#endif
