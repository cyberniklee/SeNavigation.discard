#ifndef _DWA_LOCAL_PLANNER_TRAJECTORY_SEARCH_H_
#define _DWA_LOCAL_PLANNER_TRAJECTORY_SEARCH_H_

#include "../../TrajectoryLocalPlanner/Algorithm/Trajectory.h"

namespace NS_Planner {

/**
 * @class TrajectorySearch
 * @brief Interface for modules finding a trajectory to use for navigation commands next
 */
class TrajectorySearch {
public:
  /**
   * searches the space of allowed trajectory and
   * returns one considered the optimal given the
   * constraints of the particular search.
   *
   * @param traj The container to write the result to
   * @param all_explored pass NULL or a container to collect all trajectories for debugging (has a penalty)
   */
  virtual bool findBestTrajectory(Trajectory& traj, std::vector<Trajectory>* all_explored) = 0;

  virtual ~TrajectorySearch() {}

protected:
  TrajectorySearch() {}

};


}

#endif /* TRAJECTORY_SEARCH_H_ */
