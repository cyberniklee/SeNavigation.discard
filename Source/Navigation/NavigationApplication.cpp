/*
 * NavigationApplication.cpp
 *
 *  Created on: 2016年12月3日
 *      Author: seeing
 */

#include "NavigationApplication.h"
#include "Planner/Implements/GlobalPlanner/GlobalPlanner.h"
#include <Transform/DataTypes.h>
#include <Service/ServiceType/RequestTransform.h>
#include <Service/ServiceType/ResponseTransform.h>
#include <DataSet/Dispitcher.h>
#include <DataSet/DataType/Twist.h>

namespace NS_Navigation
{
  
  NavigationApplication::NavigationApplication ()
  {
    // TODO Auto-generated constructor stub
    
  }
  
  NavigationApplication::~NavigationApplication ()
  {
    // TODO Auto-generated destructor stub
  }
  
  void
  NavigationApplication::loadParameters ()
  {
    global_planner_type_ = parameter.getParameter ("global_planner_type",
                                                   "base_global_planner");
  }
  
  bool
  NavigationApplication::makePlan (const NS_DataType::PoseStamped& goal,
                                   std::vector<NS_DataType::PoseStamped>& plan)
  {
    boost::unique_lock<NS_CostMap::Costmap2D::mutex_t> lock (
        *(global_costmap->getLayeredCostmap ()->getCostmap ()->getMutex ()));
    
    plan.clear ();
    
    //get the starting pose of the robot
    NS_Transform::Stamped < NS_Transform::Pose > global_pose;
    if (!global_costmap->getRobotPose (global_pose))
    {
      NS_NaviCommon::console.error (
          "Unable to get starting pose of robot, unable to create global plan");
      return false;
    }
    
    NS_DataType::PoseStamped start;
    NS_Transform::poseStampedTFToMsg (global_pose, start);
    
    //if the planner fails or returns a zero length plan, planning failed
    if (!global_planner->makePlan (start, goal, plan) || plan.empty ())
    {
      NS_NaviCommon::console.warning (
          "Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x,
          goal.pose.position.y);
      return false;
    }
    
    return true;
  }
  
  void
  NavigationApplication::planLoop ()
  {
    NS_NaviCommon::Rate rate (0.1);
    while (running)
    {
      rate.sleep ();
    }
  }
  
  NS_DataType::PoseStamped
  NavigationApplication::goalToGlobalFrame (NS_DataType::PoseStamped& goal)
  {
    NS_Transform::Stamped<NS_Transform::Pose> goal_pose, global_pose;
    poseStampedMsgToTF (goal, goal_pose);
    
    NS_ServiceType::RequestTransform request_odom;
    NS_ServiceType::ResponseTransform odom_transform;
    NS_ServiceType::ResponseTransform map_transform;
    
    if (service->call (NS_NaviCommon::SERVICE_TYPE_ODOMETRY_BASE_TRANSFORM,
                       &request_odom, &odom_transform) == false)
    {
      NS_NaviCommon::console.warning ("Get odometry transform failure!");
      return goal;
    }
    
    if (service->call (NS_NaviCommon::SERVICE_TYPE_MAP_ODOMETRY_TRANSFORM,
                       &request_odom, &map_transform) == false)
    {
      NS_NaviCommon::console.warning ("Get map transform failure!");
      return goal;
    }
    
    //TODO: not verify code for transform
    NS_Transform::Transform odom_tf, map_tf;
    NS_Transform::transformMsgToTF (odom_transform.transform, odom_tf);
    NS_Transform::transformMsgToTF (map_transform.transform, map_tf);
    
    global_pose.setData (odom_tf * map_tf);
    
    NS_DataType::PoseStamped global_pose_data;
    NS_Transform::poseStampedTFToMsg (global_pose, global_pose_data);
    return global_pose_data;
  }
  
  void
  NavigationApplication::goalCallback (NS_DataType::DataBase* target_goal)
  {
    NS_DataType::PoseStamped* target = (NS_DataType::PoseStamped*) target_goal;
    
    goal = *target;
    
    new_goal_trigger = true;

    if(!isQuaternionValid(goal.pose.orientation))
    {
      NS_NaviCommon::console.error("It's a illegal pose!");
      return;
    }

    if(!makePlan(*target, *global_planner_plan))
    {
      NS_NaviCommon::console.error("Make plan failure!");
      return;
    }

  }
  
  bool
  NavigationApplication::isQuaternionValid (const NS_DataType::Quaternion& q)
  {
    //first we need to check if the quaternion has nan's or infs
    if (!std::isfinite (q.x) || !std::isfinite (q.y) || !std::isfinite (q.z)
        || !std::isfinite (q.w))
    {
      NS_NaviCommon::console.error (
          "Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }
    
    NS_Transform::Quaternion tf_q (q.x, q.y, q.z, q.w);
    
    //next, we need to check if the length of the quaternion is close to zero
    if (tf_q.length2 () < 1e-6)
    {
      NS_NaviCommon::console.error (
          "Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }
    
    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    tf_q.normalize ();
    
    NS_Transform::Vector3 up (0, 0, 1);
    
    double dot = up.dot (up.rotate (tf_q.getAxis (), tf_q.getAngle ()));
    
    if (fabs (dot - 1) > 1e-3)
    {
      NS_NaviCommon::console.error (
          "Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }
    
    return true;
  }
  
  double
  NavigationApplication::distance (const NS_DataType::PoseStamped& p1,
                                   const NS_DataType::PoseStamped& p2)
  {
    return hypot (p1.pose.position.x - p2.pose.position.x,
                  p1.pose.position.y - p2.pose.position.y);
  }
  
  void
  NavigationApplication::initialize ()
  {
    loadParameters ();
    
    //set up plan triple buffer
    global_planner_plan = new std::vector<NS_DataType::PoseStamped> ();
    latest_plan = new std::vector<NS_DataType::PoseStamped> ();
    
    global_costmap = new NS_CostMap::CostmapWrapper (dispitcher, service);
    global_costmap->initialize();
    
    //load global planner
    if (global_planner_type_ == "base_global_planner")
    {
      global_planner = new NS_Planner::GlobalPlanner ();
    }
    else
    {
      global_planner = new NS_Planner::GlobalPlanner ();
    }
    
    global_planner->initialize (global_costmap);
    
    state = PLANNING;
    
    new_goal_trigger = false;
    
    initialized = true;
  }
  
  void
  NavigationApplication::run ()
  {
    running = true;
    plan_thread = boost::thread (
        boost::bind (&NavigationApplication::planLoop, this));

    global_costmap->start();
  }
  
  void
  NavigationApplication::quit ()
  {
    global_costmap->stop();

    running = false;
    plan_thread.join ();
  }

} /* namespace NS_Navigation */
