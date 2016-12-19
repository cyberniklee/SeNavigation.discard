/*
 * NavigationApplication.cpp
 *
 *  Created on: 2016年12月3日
 *      Author: seeing
 */

#include "NavigationApplication.h"
#include "Planner/Implements/BaseLocalPlanner/BaseLocalPlanner.h"
#include "Planner/Implements/DwaLocalPlanner/DwaLocalPlanner.h"
#include "Planner/Implements/GlobalPlanner/GlobalPlanner.h"
#include <Transform/DataTypes.h>
#include <Service/ServiceType/RequestTransform.h>
#include <Service/ServiceType/ResponseTransform.h>


namespace NS_Navigation {

NavigationApplication::NavigationApplication() {
	// TODO Auto-generated constructor stub

}

NavigationApplication::~NavigationApplication()
{
	// TODO Auto-generated destructor stub
}

void NavigationApplication::loadParameters()
{

}

bool NavigationApplication::makePlan(const NS_DataType::PoseStamped& goal, std::vector<NS_DataType::PoseStamped>& plan)
{
  boost::unique_lock<NS_CostMap::Costmap2D::mutex_t> lock(*(global_costmap->getLayeredCostmap()->getCostmap()->getMutex()));

  plan.clear();

  //get the starting pose of the robot
  NS_Transform::Stamped<NS_Transform::Pose> global_pose;
  if(!global_costmap->getRobotPose(global_pose))
  {
    NS_NaviCommon::console.error("Unable to get starting pose of robot, unable to create global plan");
    return false;
  }

  NS_DataType::PoseStamped start;
  NS_Transform::poseStampedTFToMsg(global_pose, start);

  //if the planner fails or returns a zero length plan, planning failed
  if(!global_planner->makePlan(start, goal, plan) || plan.empty())
  {
    NS_NaviCommon::console.warning("Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
    return false;
  }

  return true;
}

void NavigationApplication::planLoop()
{
  while(running)
  {

  }
}

NS_DataType::PoseStamped NavigationApplication::goalToGlobalFrame(NS_DataType::PoseStamped& goal)
{
  NS_Transform::Stamped<NS_Transform::Pose> goal_pose, global_pose;
  poseStampedMsgToTF(goal, goal_pose);

  NS_ServiceType::RequestTransform request_odom;
  NS_ServiceType::ResponseTransform odom_transform;
  NS_ServiceType::ResponseTransform map_transform;

  if(service->call(NS_NaviCommon::SERVICE_TYPE_ODOMETRY_BASE_TRANSFORM, &request_odom, &odom_transform) == false)
  {
	NS_NaviCommon::console.warning("Get odometry transform failure!");
	return goal;
  }

  if(service->call(NS_NaviCommon::SERVICE_TYPE_MAP_ODOMETRY_TRANSFORM, &request_odom, &map_transform) == false)
  {
  	NS_NaviCommon::console.warning("Get map transform failure!");
  	return goal;
  }

  //TODO: not verify code for transform
  NS_Transform::Transform odom_tf, map_tf;
  NS_Transform::transformMsgToTF(odom_transform.transform, odom_tf);
  NS_Transform::transformMsgToTF(map_transform.transform, map_tf);

  global_pose.setData(odom_tf * map_tf);

  NS_DataType::PoseStamped global_pose_data;
  NS_Transform::poseStampedTFToMsg(global_pose, global_pose_data);
  return global_pose_data;
}

void NavigationApplication::goalCallback(NS_DataType::DataBase* target_goal)
{
  NS_DataType::PoseStamped* goal = (NS_DataType::PoseStamped*)target_goal;
}

void NavigationApplication::initialize()
{
  loadParameters();

  //set up plan triple buffer
  global_planner_plan = new std::vector<NS_DataType::PoseStamped>();
  latest_plan = new std::vector<NS_DataType::PoseStamped>();
  local_planner_plan = new std::vector<NS_DataType::PoseStamped>();

  global_costmap = new NS_CostMap::CostmapWrapper(dispitcher, service);
  local_costmap = new NS_CostMap::CostmapWrapper(dispitcher, service);

  //load global planner
  if(global_planner_type_ == "base_global_planner")
  {
    global_planner = new NS_Planner::GlobalPlanner();
  }else{
    global_planner = new NS_Planner::GlobalPlanner();
  }

  //load local planner
  if(local_planner_type_ == "base_local_planner")
  {
    local_planner = new NS_Planner::BaseLocalPlanner();
  }else if(local_planner_type_ == "dwa_local_planner")
  {
    local_planner = new NS_Planner::DwaLocalPlanner();
  }else{
    local_planner = new NS_Planner::BaseLocalPlanner();
  }

  global_planner->initialize(global_costmap, dispitcher, service);
  local_planner->initialize(local_costmap, dispitcher, service);

  state = PLANNING;

  initialized = true;
}

void NavigationApplication::run()
{
  running = true;
  plan_thread = boost::thread(boost::bind(&NavigationApplication::planLoop, this));
}

void NavigationApplication::quit()
{
  running = false;
  plan_thread.join();
}

} /* namespace NS_Navigation */
