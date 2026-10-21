#include "trajectory.h"
#include <cassert>
int main() {
  TrajectoryConfig c; assert(c.valid());
  float t=0, ds=0, theta=0; int phases=0;
  for(;t<c.duration();t+=.001f) {
    auto command=trajectoryCommand(c,t);
    ds+=command.velocity*.001f; theta+=command.yawRate*.001f;
    phases|=1<<(command.phase-10);
  }
  assert(phases==127);
  assert(fabsf(ds-1)<.001f && fabsf(theta-c.turnAngle)<.001f);
  assert(trajectoryCommand(c,c.initialRest).phase==11);
  assert(trajectoryCommand(c,c.duration()+.001f).phase==0);
  c.turnAngle=-c.turnAngle;
  assert(trajectoryCommand(c,9).yawRate<0);
  c.speed=0; assert(!c.valid()); assert(trajectoryCommand(c,5).phase==0);
}
