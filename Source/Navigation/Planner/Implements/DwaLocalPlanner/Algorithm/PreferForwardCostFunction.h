#ifndef _DWA_LOCAL_PLANNER_PREFER_FORWARD_COST_FUNCTION_H_
#define _DWA_LOCAL_PLANNER_PREFER_FORWARD_COST_FUNCTION_H_

#include "TrajectoryCostFunction.h"

namespace NS_Planner
{
  
  class PreferForwardCostFunction: public TrajectoryCostFunction
  {
  public:
    
    PreferForwardCostFunction (double penalty)
        : penalty_ (penalty)
    {
    }
    ~PreferForwardCostFunction ()
    {
    }
    
    double
    scoreTrajectory (Trajectory &traj);

    bool
    prepare ()
    {
      return true;
    }
    ;

    void
    setPenalty (double penalty)
    {
      penalty_ = penalty;
    }
    
  private:
    double penalty_;
  };

} /* namespace base_local_planner */
#endif /* PREFER_FORWARD_COST_FUNCTION_H_ */
