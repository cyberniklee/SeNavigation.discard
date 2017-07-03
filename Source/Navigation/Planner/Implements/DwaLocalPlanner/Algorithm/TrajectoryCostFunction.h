
#ifndef _DWA_LOCAL_PLANNER_TRAJECTORY_COST_FUNCTION_H_
#define _DWA_LOCAL_PLANNER_TRAJECTORY_COST_FUNCTION_H_

#include "../../TrajectoryLocalPlanner/Algorithm/Trajectory.h"

namespace NS_Planner {

/**
 * @class TrajectoryCostFunction
 * @brief Provides an interface for critics of trajectories
 * During each sampling run, a batch of many trajectories will be scored using such a cost function.
 * The prepare method is called before each batch run, and then for each
 * trajectory of the sampling set, score_trajectory may be called.
 */
class TrajectoryCostFunction {
public:

  /**
   *
   * General updating of context values if required.
   * Subclasses may overwrite. Return false in case there is any error.
   */
  virtual bool prepare() = 0;

  /**
   * return a score for trajectory traj
   */
  virtual double scoreTrajectory(Trajectory &traj) = 0;

  double getScale() {
    return scale_;
  }

  void setScale(double scale) {
    scale_ = scale;
  }

  virtual ~TrajectoryCostFunction() {}

protected:
  TrajectoryCostFunction(double scale = 1.0): scale_(scale) {}

private:
  double scale_;
};

}

#endif /* TRAJECTORYCOSTFUNCTION_H_ */
