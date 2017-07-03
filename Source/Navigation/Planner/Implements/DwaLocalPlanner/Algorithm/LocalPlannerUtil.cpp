
#include "LocalPlannerUtil.h"

#include "../../TrajectoryLocalPlanner/Algorithm/GoalFunctions.h"

#include <Console/Console.h>

namespace NS_Planner {

void LocalPlannerUtil::initialize(NS_NaviCommon::Service* service,
                                  NS_CostMap::Costmap2D* costmap) {
  if(!initialized_) {
    service_ = service;
    costmap_ = costmap;
    initialized_ = true;
  }
  else{
    NS_NaviCommon::console.warning("Planner utils have already been initialized, doing nothing.");
  }
}

void LocalPlannerUtil::reconfigureCB(LocalPlannerLimits &config, bool restore_defaults)
{
  if(setup_ && restore_defaults) {
    config = default_limits_;
  }

  if(!setup_) {
    default_limits_ = config;
    setup_ = true;
  }
  boost::mutex::scoped_lock l(limits_configuration_mutex_);
  limits_ = LocalPlannerLimits(config);
}

NS_CostMap::Costmap2D* LocalPlannerUtil::getCostmap() {
  return costmap_;
}

LocalPlannerLimits LocalPlannerUtil::getCurrentLimits() {
  boost::mutex::scoped_lock l(limits_configuration_mutex_);
  return limits_;
}


bool LocalPlannerUtil::getGoal(NS_Transform::Stamped<NS_Transform::Pose>& goal_pose) {
  //we assume the global goal is the last point in the global plan
  return NS_Planner::getGoalPose(service_,
        global_plan_,
        goal_pose);
}

bool LocalPlannerUtil::setPlan(const std::vector<NS_DataType::PoseStamped>& orig_global_plan) {
  if(!initialized_){
    NS_NaviCommon::console.error("Planner utils have not been initialized, please call initialize() first");
    return false;
  }

  //reset the global plan
  global_plan_.clear();

  global_plan_ = orig_global_plan;

  return true;
}

bool LocalPlannerUtil::getLocalPlan(NS_Transform::Stamped<NS_Transform::Pose>& global_pose, std::vector<NS_DataType::PoseStamped>& transformed_plan) {
  //get the global plan in our frame
  if(!NS_Planner::transformGlobalPlan(
      service_,
      global_plan_,
      global_pose,
      *costmap_,
      transformed_plan)) {
    NS_NaviCommon::console.warning("Could not transform the global plan to the frame of the controller");
    return false;
  }

  //now we'll prune the plan based on the position of the robot
  if(limits_.prune_plan) {
    NS_Planner::prunePlan(global_pose, transformed_plan, global_plan_);
  }
  return true;
}




} // namespace
