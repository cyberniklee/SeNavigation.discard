
#ifndef _DWA_LOCAL_PLANNER_OBSTACLE_COST_FUNCTION_H_
#define _DWA_LOCAL_PLANNER_OBSTACLE_COST_FUNCTION_H_

#include "TrajectoryCostFunction.h"

#include "../../TrajectoryLocalPlanner/Algorithm/CostmapModel.h"
#include "../../../../CostMap/CostMap2D/CostMap2D.h"

namespace NS_Planner {

/**
 * class ObstacleCostFunction
 * @brief Uses costmap 2d to assign negative costs if robot footprint
 * is in obstacle on any point of the trajectory.
 */
class ObstacleCostFunction : public TrajectoryCostFunction {

public:
  ObstacleCostFunction(NS_CostMap::Costmap2D* costmap);
  ~ObstacleCostFunction();

  bool prepare();
  double scoreTrajectory(Trajectory &traj);

  void setSumScores(bool score_sums){ sum_scores_=score_sums; }

  void setParams(double max_trans_vel, double max_scaling_factor, double scaling_speed);
  void setFootprint(std::vector<NS_DataType::Point> footprint_spec);

  // helper functions, made static for easy unit testing
  static double getScalingFactor(Trajectory &traj, double scaling_speed, double max_trans_vel, double max_scaling_factor);
  static double footprintCost(
      const double& x,
      const double& y,
      const double& th,
      double scale,
      std::vector<NS_DataType::Point> footprint_spec,
      NS_CostMap::Costmap2D* costmap,
      NS_Planner::WorldModel* world_model);

private:
  NS_CostMap::Costmap2D* costmap_;
  std::vector<NS_DataType::Point> footprint_spec_;
  NS_Planner::WorldModel* world_model_;
  double max_trans_vel_;
  bool sum_scores_;
  //footprint scaling with velocity;
  double max_scaling_factor_, scaling_speed_;
};

} /* namespace base_local_planner */
#endif /* OBSTACLE_COST_FUNCTION_H_ */
