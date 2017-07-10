#ifndef _DWA_LOCAL_PLANNER_SIMPLE_SCORED_SAMPLING_PLANNER_H_
#define _DWA_LOCAL_PLANNER_SIMPLE_SCORED_SAMPLING_PLANNER_H_

#include <vector>
#include "../../TrajectoryLocalPlanner/Algorithm/Trajectory.h"
#include "TrajectoryCostFunction.h"
#include "TrajectorySampleGenerator.h"
#include "TrajectorySearch.h"

namespace NS_Planner
{
  
  /**
   * @class SimpleScoredSamplingPlanner
   * @brief Generates a local plan using the given generator and cost functions.
   * Assumes less cost are best, and negative costs indicate infinite costs
   *
   * This is supposed to be a simple and robust implementation of
   * the TrajectorySearch interface. More efficient search may well be
   * possible using search heuristics, parallel search, etc.
   */
  class SimpleScoredSamplingPlanner: public TrajectorySearch
  {
  public:
    
    ~SimpleScoredSamplingPlanner ()
    {
    }
    
    SimpleScoredSamplingPlanner ()
    {
    }
    
    /**
     * Takes a list of generators and critics. Critics return costs > 0, or negative costs for invalid trajectories.
     * Generators other than the first are fallback generators,  meaning they only get to generate if the previous
     * generator did not find a valid trajectory.
     * Will use every generator until it stops returning trajectories or count reaches max_samples.
     * Then resets count and tries for the next in the list.
     * passing max_samples = -1 (default): Each Sampling planner will continue to call
     * generator until generator runs out of samples (or forever if that never happens)
     */
    SimpleScoredSamplingPlanner (
        std::vector<TrajectorySampleGenerator*> gen_list,
        std::vector<TrajectoryCostFunction*>& critics, int max_samples = -1);

    /**
     * runs all scoring functions over the trajectory creating a weigthed sum
     * of positive costs, aborting as soon as a negative cost are found or costs greater
     * than positive best_traj_cost accumulated
     */
    double
    scoreTrajectory (Trajectory& traj, double best_traj_cost);

    /**
     * Calls generator until generator has no more samples or max_samples is reached.
     * For each generated traj, calls critics in turn. If any critic returns negative
     * value, that value is assumed as costs, else the costs are the sum of all critics
     * result. Returns true and sets the traj parameter to the first trajectory with
     * minimal non-negative costs if sampling yields trajectories with non-negative costs,
     * else returns false.
     *
     * @param traj The container to write the result to
     * @param all_explored pass NULL or a container to collect all trajectories for debugging (has a penalty)
     */
    bool
    findBestTrajectory (Trajectory& traj,
                        std::vector<Trajectory>* all_explored = 0);

  private:
    std::vector<TrajectorySampleGenerator*> gen_list_;
    std::vector<TrajectoryCostFunction*> critics_;

    int max_samples_;
  };

} // namespace

#endif /* SIMPLE_SCORED_SAMPLING_PLANNER_H_ */
