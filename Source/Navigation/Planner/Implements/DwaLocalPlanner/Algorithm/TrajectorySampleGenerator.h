#ifndef _DWA_LOCAL_PLANNER_TRAJECTORY_SAMPLE_GENERATOR_H_
#define _DWA_LOCAL_PLANNER_TRAJECTORY_SAMPLE_GENERATOR_H_

#include "../../TrajectoryLocalPlanner/Algorithm/Trajectory.h"

namespace NS_Planner {

/**
 * @class TrajectorySampleGenerator
 * @brief Provides an interface for navigation trajectory generators
 */
class TrajectorySampleGenerator {
public:

  /**
   * Whether this generator can create more trajectories
   */
  virtual bool hasMoreTrajectories() = 0;

  /**
   * Whether this generator can create more trajectories
   */
  virtual bool nextTrajectory(Trajectory &traj) = 0;

  /**
   * @brief  Virtual destructor for the interface
   */
  virtual ~TrajectorySampleGenerator() {}

protected:
  TrajectorySampleGenerator() {}

};

} // end namespace

#endif /* TRAJECTORY_SAMPLE_GENERATOR_H_ */
