#include "PreferForwardCostFunction.h"

#include <math.h>

namespace NS_Planner
{
  
  double
  PreferForwardCostFunction::scoreTrajectory (Trajectory &traj)
  {
    // backward motions bad on a robot without backward sensors
    if (traj.xv_ < 0.0)
    {
      return penalty_;
    }
    // strafing motions also bad on such a robot
    if (traj.xv_ < 0.1 && fabs (traj.thetav_) < 0.2)
    {
      return penalty_;
    }
    // the more we rotate, the less we progress forward
    return fabs (traj.thetav_) * 10;
  }

} /* namespace base_local_planner */
