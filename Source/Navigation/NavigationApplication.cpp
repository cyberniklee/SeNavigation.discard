/*
 * NavigationApplication.cpp
 *
 *  Created on: 2016年12月3日
 *      Author: seeing
 */

#include "NavigationApplication.h"
#include "Planner/Implements/GlobalPlanner/GlobalPlanner.h"
#include "Planner/Implements/TrajectoryLocalPlanner/TrajectoryLocalPlanner.h"
#include "Planner/Implements/DwaLocalPlanner/DwaLocalPlanner.h"
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
    new_goal_trigger = false;
  }
  
  NavigationApplication::~NavigationApplication ()
  {
    // TODO Auto-generated destructor stub
  }
  
  void
  NavigationApplication::loadParameters ()
  {
    global_planner_type_ = parameter.getParameter ("global_planner_type",
                                                   "global_planner");
    
    local_planner_type_ = parameter.getParameter ("local_planner_type",
                                                  "trajectory_local_planner");

    planner_frequency_ = parameter.getParameter ("planner_frequency", 0.0f);
    controller_frequency_ = parameter.getParameter ("controller_frequency", 10.0f);
  }
  
  bool
  NavigationApplication::makePlan (const NS_DataType::PoseStamped& goal,
                                   std::vector<NS_DataType::PoseStamped>& plan)
  {
    boost::unique_lock<NS_CostMap::Costmap2D::mutex_t> lock (
        *(global_costmap->getLayeredCostmap ()->getCostmap ()->getMutex ()));
    
    plan.clear ();
    
    //get the starting pose of the robot
    NS_Transform::Stamped<NS_Transform::Pose> global_pose;
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
    
    NS_NaviCommon::console.debug ("Plans computed, %d points to go...", plan.size ());
    for (size_t i = 0; i < plan.size (); i++)
    {
      NS_NaviCommon::console.debug ("[%d] x = %lf, y = %lf", (i + 1), plan[i].pose.position.x, plan[i].pose.position.y);
    }

    return true;
  }
  
  void
  NavigationApplication::runRecovery ()
  {

  }

  void
  NavigationApplication::resetState ()
  {
    state = PLANNING;
    publishZeroVelocity ();
  }

  void
  NavigationApplication::controlLoop ()
  {
    while (running)
    {
      NS_NaviCommon::Rate rate (controller_frequency_);
      controller_mutex.lock ();
      while ((state != CONTROLLING || global_planner_plan->size () == 0) && running)
      {
        controller_cond.timed_wait (controller_mutex,
                              (boost::get_system_time () + boost::posix_time::milliseconds (PLANNER_LOOP_TIMEOUT)));
      }
      controller_mutex.unlock ();

      if (!running)
      {
        NS_NaviCommon::console.message ("Quit local planning loop...");
        break;
      }

      if (!local_planner->setPlan (*global_planner_plan))
      {
        NS_NaviCommon::console.error ("Set plan to local planner failure!");
        resetState ();
        continue;
      }

      while (running)
      {
        NS_DataType::Twist cmd_vel;
        NS_NaviCommon::Time last_valid_control;

        //update feedback to correspond to our curent position
        NS_Transform::Stamped<NS_Transform::Pose> global_pose;
        global_costmap->getRobotPose (global_pose);
        NS_DataType::PoseStamped current_position;
        NS_Transform::poseStampedTFToMsg (global_pose, current_position);

        if (distance (current_position, oscillation_pose_) >= oscillation_distance_)
        {
          //TODO: oscillation

          oscillation_pose_ = current_position;
        }

        switch (state)
        {
          case PLANNING:break;
          case CONTROLLING:
            if (local_planner->isGoalReached ())
            {
              NS_NaviCommon::console.message ("The goal has reached!");
              resetState ();
              break;
            }

            //TODO : check oscillation and clear

            if (local_planner->computeVelocityCommands (cmd_vel))
            {
              NS_NaviCommon::console.debug ("Got velocity data : l_x=%.3lf, l_y=%.3lf, a_z=%.3lf!",
                                            cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
              last_valid_control = NS_NaviCommon::Time::now ();
              publishVelocity (cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
            }
            else
            {
              NS_NaviCommon::console.warning ("The planner can not got a valid velocity data!");
              NS_NaviCommon::Time next_control = last_valid_control + NS_NaviCommon::Duration (controller_patience_);

              if (NS_NaviCommon::Time::now () > next_control)
              {
                publishZeroVelocity ();
                state = CLEARING;
                resetState ();
              }
              else
              {
                //TODO: re-plan

                publishZeroVelocity ();
                state = PLANNING;
                resetState ();
              }
            }

            break;
          case CLEARING:
            runRecovery ();
            state = PLANNING;
            break;
        }

        rate.sleep ();
      }
    }
  }

  void
  NavigationApplication::planLoop ()
  {
    NS_NaviCommon::Rate rate (planner_frequency_);

    while (running)
    {
      planner_mutex.lock ();
      while (!new_goal_trigger && running)
      {
        planner_cond.timed_wait (planner_mutex,
                                 (boost::get_system_time () + boost::posix_time::milliseconds (PLANNER_LOOP_TIMEOUT)));
      }
      planner_mutex.unlock ();

      if (!running)
      {
        NS_NaviCommon::console.message ("Quit global planning loop...");
        break;
      }

      new_goal_trigger = false;

      if (!makePlan (goal, *latest_plan) && state != PLANNING)
      {
        NS_NaviCommon::console.error ("Make plan failure!");
        continue;
      }

      controller_mutex.lock ();
      state = CONTROLLING;
      global_planner_plan->clear ();
      global_planner_plan->assign (latest_plan->begin (), latest_plan->end ());
      controller_cond.notify_one ();
      controller_mutex.unlock ();

      if (planner_frequency_ != 0.0f)
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
    if (!isQuaternionValid (target->pose.orientation))
    {
      NS_NaviCommon::console.error ("It's a illegal pose!");
      return;
    }

    NS_DataType::PoseStamped new_goal = goalToGlobalFrame (*target);

    planner_mutex.lock ();
    goal = new_goal;
    new_goal_trigger = true;
    state = PLANNING;
    planner_cond.notify_one ();
    planner_mutex.unlock();

    delete target_goal;

  }
  
  void
  NavigationApplication::publishZeroVelocity ()
  {
    publishVelocity(0, 0, 0);
  }

  void
  NavigationApplication::publishVelocity (double linear_x, double linear_y, double angular_z)
  {
    NS_DataType::Twist* vel = new NS_DataType::Twist;
    vel->linear.x = linear_x;
    vel->linear.y = linear_y;
    vel->angular.z = angular_z;
    dispitcher->publish (NS_NaviCommon::DATA_TYPE_TWIST, vel);
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
    

    /*
     * make global planner and global costmap
     */
    global_costmap = new NS_CostMap::CostmapWrapper (dispitcher, service);
    global_costmap->initialize ();
    
    //load global planner
    if (global_planner_type_ == "global_planner")
    {
      global_planner = new NS_Planner::GlobalPlanner ();
    }
    else
    {
      global_planner = new NS_Planner::GlobalPlanner ();
    }
    
    global_planner->initialize (global_costmap, dispitcher, service);

    /*
     * make local planner and local costmap
     */
    local_costmap = new NS_CostMap::CostmapWrapper (dispitcher, service);
    local_costmap->initialize ();

    //load local planner
    if (local_planner_type_ == "trajectory_local_planner")
    {
      local_planner = new NS_Planner::TrajectoryLocalPlanner ();
    }
    else if (local_planner_type_ == "dwa_local_planner")
    {
      local_planner = new NS_Planner::DwaLocalPlanner ();
    }
    else
    {
      local_planner = new NS_Planner::TrajectoryLocalPlanner ();
    }
    
    local_planner->initialize (global_costmap, dispitcher, service);

    dispitcher->subscribe (
        NS_NaviCommon::DATA_TYPE_TARGET_GOAL,
        boost::bind (&NavigationApplication::goalCallback, this, _1));

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

    control_thread = boost::thread (
        boost::bind (&NavigationApplication::controlLoop, this));
    
    global_costmap->start ();
    local_costmap->start();
  }
  
  void
  NavigationApplication::quit ()
  {
    global_costmap->stop ();
    local_costmap->stop ();
    
    running = false;
    plan_thread.join ();
  }

} /* namespace NS_Navigation */
