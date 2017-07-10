#ifndef _DWA_LOCAL_PLANNER_MAP_GRID_COST_FUNCTION_H_
#define _DWA_LOCAL_PLANNER_MAP_GRID_COST_FUNCTION_H_

#include "TrajectoryCostFunction.h"

#include "../../../../CostMap/CostMap2D/CostMap2D.h"
#include "../../TrajectoryLocalPlanner/Algorithm/MapGrid.h"

namespace NS_Planner
{
  
  /**
   * when scoring a trajectory according to the values in mapgrid, we can take
   *return the value of the last point (if no of the earlier points were in
   * return collision), the sum for all points, or the product of all (non-zero) points
   */
  enum CostAggregationType
  {
    Last, Sum, Product
  };
  
  /**
   * This class provides cost based on a map_grid of a small area of the world.
   * The map_grid covers a the costmap, the costmap containing the information
   * about sensed obstacles. The map_grid is used by setting
   * certain cells to distance 0, and then propagating distances around them,
   * filling up the area reachable around them.
   *
   * The approach using grid_maps is used for computational efficiency, allowing to
   * score hundreds of trajectories very quickly.
   *
   * This can be used to favor trajectories which stay on a given path, or which
   * approach a given goal.
   * @param costmap_ros Reference to object giving updates of obstacles around robot
   * @param xshift where the scoring point is with respect to robot center pose
   * @param yshift where the scoring point is with respect to robot center pose
   * @param is_local_goal_function, scores for local goal rather than whole path
   * @param aggregationType how to combine costs along trajectory
   */
  class MapGridCostFunction: public TrajectoryCostFunction
  {
  public:
    MapGridCostFunction (NS_CostMap::Costmap2D* costmap, double xshift = 0.0,
                         double yshift = 0.0, bool is_local_goal_function =
                             false,
                         CostAggregationType aggregationType = Last);

    ~MapGridCostFunction ()
    {
    }
    
    /**
     * set line segments on the grid with distance 0, resets the grid
     */
    void
    setTargetPoses (std::vector<NS_DataType::PoseStamped> target_poses);

    void
    setXShift (double xshift)
    {
      xshift_ = xshift;
    }
    void
    setYShift (double yshift)
    {
      yshift_ = yshift;
    }
    
    /** @brief If true, failures along the path cause the entire path to be rejected.
     *
     * Default is true. */
    void
    setStopOnFailure (bool stop_on_failure)
    {
      stop_on_failure_ = stop_on_failure;
    }
    
    /**
     * propagate distances
     */
    bool
    prepare ();

    double
    scoreTrajectory (Trajectory &traj);

    /**
     * return a value that indicates cell is in obstacle
     */
    double
    obstacleCosts ()
    {
      return map_.obstacleCosts ();
    }
    
    /**
     * returns a value indicating cell was not reached by wavefront
     * propagation of set cells. (is behind walls, regarding the region covered by grid)
     */
    double
    unreachableCellCosts ()
    {
      return map_.unreachableCellCosts ();
    }
    
    // used for easier debugging
    double
    getCellCosts (unsigned int cx, unsigned int cy);

  private:
    std::vector<NS_DataType::PoseStamped> target_poses_;
    NS_CostMap::Costmap2D* costmap_;

    NS_Planner::MapGrid map_;
    CostAggregationType aggregationType_;
    /// xshift and yshift allow scoring for different
    // ooints of robots than center, like fron or back
    // this can help with alignment or keeping specific
    // wheels on tracks both default to 0
    double xshift_;
    double yshift_;
    // if true, we look for a suitable local goal on path, else we use the full path for costs
    bool is_local_goal_function_;
    bool stop_on_failure_;
  };

} /* namespace base_local_planner */
#endif /* MAP_GRID_COST_FUNCTION_H_ */
