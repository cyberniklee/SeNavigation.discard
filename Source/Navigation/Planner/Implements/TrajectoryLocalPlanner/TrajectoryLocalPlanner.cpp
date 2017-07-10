#include "TrajectoryLocalPlanner.h"

#include <sys/time.h>
#include <boost/tokenizer.hpp>

#include <Eigen/Core>
#include <cmath>

#include <Console/Console.h>

#include "Algorithm/GoalFunctions.h"
#include <DataSet/DataType/Path.h>

#include <Service/Service.h>
#include <Parameter/Parameter.h>

#include <Service/ServiceType/RequestOdometry.h>
#include <Service/ServiceType/ResponseOdometry.h>

namespace NS_Planner
{
  
  TrajectoryLocalPlanner::TrajectoryLocalPlanner ()
  {
    world_model_ = NULL;
    tc_ = NULL;
    setup_ = false;
    initialized_ = false;
    odom_helper_ = NULL;
  }
  
  void
  TrajectoryLocalPlanner::onInitialize ()
  {
    if (!isInitialized ())
    {
      NS_NaviCommon::Parameter parameter;
      
      rot_stopped_velocity_ = 1e-2;
      trans_stopped_velocity_ = 1e-2;
      double sim_time, sim_granularity, angular_sim_granularity;
      int vx_samples, vtheta_samples;
      double pdist_scale, gdist_scale, occdist_scale, heading_lookahead,
          oscillation_reset_dist, escape_reset_dist, escape_reset_theta;
      bool holonomic_robot, dwa, simple_attractor, heading_scoring;
      double heading_scoring_timestep;
      double max_vel_x, min_vel_x;
      double backup_vel;
      double stop_time_buffer;
      std::string world_model_type;
      rotating_to_goal_ = false;
      
      //initialize the copy of the costmap the controller will use
      costmap_ = costmap->getCostmap ();
      
      /*
       global_frame_ = costmap_ros_->getGlobalFrameID();
       robot_base_frame_ = costmap_ros_->getBaseFrameID();
       */
      parameter.loadConfigurationFile ("trajectory_local_planner.xml");
      
      if (parameter.getParameter ("prune_plan", 1) == 1)
      {
        prune_plan_ = true;
      }
      else
      {
        prune_plan_ = false;
      }
      
      yaw_goal_tolerance_ = parameter.getParameter ("yaw_goal_tolerance",
                                                    0.05f);
      xy_goal_tolerance_ = parameter.getParameter ("xy_goal_tolerance", 0.10f);
      acc_lim_x_ = parameter.getParameter ("acc_lim_x", 2.5f);
      acc_lim_y_ = parameter.getParameter ("acc_lim_y", 2.5f);
      acc_lim_theta_ = parameter.getParameter ("acc_lim_theta", 3.2f);
      
      stop_time_buffer = parameter.getParameter ("stop_time_buffer", 0.2f);
      
      if (parameter.getParameter ("latch_xy_goal_tolerance", 0) == 1)
      {
        latch_xy_goal_tolerance_ = true;
      }
      else
      {
        latch_xy_goal_tolerance_ = false;
      }
      
      sim_period_ = parameter.getParameter ("sim_period", 0.05f);
      NS_NaviCommon::console.debug ("Sim period is set to %.2f", sim_period_);
      
      sim_time = parameter.getParameter ("sim_time", 1.0f);
      sim_granularity = parameter.getParameter ("sim_granularity", 0.025f);
      angular_sim_granularity = parameter.getParameter (
          "angular_sim_granularity", 0.025f);
      vx_samples = parameter.getParameter ("vx_samples", 3);
      vtheta_samples = parameter.getParameter ("vtheta_samples", 20);
      
      pdist_scale = parameter.getParameter ("path_distance_bias", 0.6f);
      gdist_scale = parameter.getParameter ("goal_distance_bias", 0.8f);
      occdist_scale = parameter.getParameter ("occdist_scale", 0.01f);
      
      bool meter_scoring;
      if (parameter.getParameter ("meter_scoring", 1) == 1)
      {
        meter_scoring = true;
      }
      else
      {
        meter_scoring = false;
      }
      
      if (meter_scoring)
      {
        //if we use meter scoring, then we want to multiply the biases by the resolution of the costmap
        double resolution = costmap_->getResolution ();
        gdist_scale *= resolution;
        pdist_scale *= resolution;
        occdist_scale *= resolution;
      }
      else
      {
        NS_NaviCommon::console.warning (
            "Trajectory Rollout planner initialized with param meter_scoring set to false. Set it to true to make your settins robust against changes of costmap resolution.");
      }
      
      heading_lookahead = parameter.getParameter ("heading_lookahead", 0.325f);
      oscillation_reset_dist = parameter.getParameter ("oscillation_reset_dist",
                                                       0.05f);
      escape_reset_dist = parameter.getParameter ("escape_reset_dist", 0.10f);
      escape_reset_theta = parameter.getParameter ("escape_reset_theta",
                                                   (float) M_PI_4);
      if (parameter.getParameter ("holonomic_robot", 1) == 1)
      {
        holonomic_robot = true;
      }
      else
      {
        holonomic_robot = false;
      }
      max_vel_x = parameter.getParameter ("max_vel_x", 0.5f);
      min_vel_x = parameter.getParameter ("min_vel_x", 0.1f);
      
      double max_rotational_vel;
      max_rotational_vel = parameter.getParameter ("max_rotational_vel", 1.0f);
      max_vel_th_ = max_rotational_vel;
      min_vel_th_ = -1.0 * max_rotational_vel;
      min_in_place_vel_th_ = parameter.getParameter (
          "min_in_place_rotational_vel", 0.4f);
      reached_goal_ = false;
      backup_vel = -0.1;
      backup_vel = parameter.getParameter ("escape_vel", -0.1f);
      
      if (backup_vel >= 0.0)
        NS_NaviCommon::console.warning (
            "You've specified a positive escape velocity. This is probably not what you want and will cause the robot to move forward instead of backward. You should probably change your escape_vel parameter to be negative");
      
      if (parameter.getParameter ("dwa", 1) == 1)
      {
        dwa = true;
      }
      else
      {
        dwa = false;
      }
      if (parameter.getParameter ("heading_scoring", 0) == 1)
      {
        heading_scoring = true;
      }
      else
      {
        heading_scoring = false;
      }
      heading_scoring_timestep = parameter.getParameter (
          "heading_scoring_timestep", 0.8f);
      
      simple_attractor = false;
      
      world_model_ = new CostmapModel (*costmap_);
      
      footprint_spec_ = costmap->getRobotFootprint ();
      
      odom_helper_ = new OdometryHelper (service);
      
      std::vector<double> y_vels = std::vector<double> (0);
      y_vels.clear ();
      
      tc_ = new TrajectoryPlanner (*world_model_, *costmap_, footprint_spec_,
                                   acc_lim_x_, acc_lim_y_, acc_lim_theta_,
                                   sim_time, sim_granularity, vx_samples,
                                   vtheta_samples, pdist_scale, gdist_scale,
                                   occdist_scale, heading_lookahead,
                                   oscillation_reset_dist, escape_reset_dist,
                                   escape_reset_theta, holonomic_robot,
                                   max_vel_x, min_vel_x, max_vel_th_,
                                   min_vel_th_, min_in_place_vel_th_,
                                   backup_vel, dwa, heading_scoring,
                                   heading_scoring_timestep, meter_scoring,
                                   simple_attractor, y_vels, stop_time_buffer,
                                   sim_period_, angular_sim_granularity);
      
      initialized_ = true;
      
    }
    else
    {
      NS_NaviCommon::console.warning (
          "This planner has already been initialized, doing nothing");
    }
  }
  
  TrajectoryLocalPlanner::~TrajectoryLocalPlanner ()
  {
    //make sure to clean things up
    if (tc_ != NULL)
      delete tc_;
    
    if (world_model_ != NULL)
      delete world_model_;
    
    if (odom_helper_ != NULL)
      delete odom_helper_;
  }
  
  bool
  TrajectoryLocalPlanner::stopWithAccLimits (
      const NS_Transform::Stamped<NS_Transform::Pose>& global_pose,
      const NS_Transform::Stamped<NS_Transform::Pose>& robot_vel,
      NS_DataType::Twist& cmd_vel)
  {
    //slow down with the maximum possible acceleration... we should really use the frequency that we're running at to determine what is feasible
    //but we'll use a tenth of a second to be consistent with the implementation of the local planner.
    double vx = sign (robot_vel.getOrigin ().x ())
        * std::max (
            0.0,
            (fabs (robot_vel.getOrigin ().x ()) - acc_lim_x_ * sim_period_));
    double vy = sign (robot_vel.getOrigin ().y ())
        * std::max (
            0.0,
            (fabs (robot_vel.getOrigin ().y ()) - acc_lim_y_ * sim_period_));
    
    double vel_yaw = NS_Transform::getYaw (robot_vel.getRotation ());
    double vth = sign (vel_yaw)
        * std::max (0.0, (fabs (vel_yaw) - acc_lim_theta_ * sim_period_));
    
    //we do want to check whether or not the command is valid
    double yaw = NS_Transform::getYaw (global_pose.getRotation ());
    bool valid_cmd = tc_->checkTrajectory (global_pose.getOrigin ().getX (),
                                           global_pose.getOrigin ().getY (),
                                           yaw, robot_vel.getOrigin ().getX (),
                                           robot_vel.getOrigin ().getY (),
                                           vel_yaw, vx, vy, vth);
    
    //if we have a valid command, we'll pass it on, otherwise we'll command all zeros
    if (valid_cmd)
    {
      NS_NaviCommon::console.debug (
          "Slowing down... using vx, vy, vth: %.2f, %.2f, %.2f", vx, vy, vth);
      cmd_vel.linear.x = vx;
      cmd_vel.linear.y = vy;
      cmd_vel.angular.z = vth;
      return true;
    }
    
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    return false;
  }
  
  bool
  TrajectoryLocalPlanner::rotateToGoal (
      const NS_Transform::Stamped<NS_Transform::Pose>& global_pose,
      const NS_Transform::Stamped<NS_Transform::Pose>& robot_vel,
      double goal_th, NS_DataType::Twist& cmd_vel)
  {
    double yaw = NS_Transform::getYaw (global_pose.getRotation ());
    double vel_yaw = NS_Transform::getYaw (robot_vel.getRotation ());
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    double ang_diff = NS_Geometry::NS_Angles::shortest_angular_distance (
        yaw, goal_th);
    
    double v_theta_samp =
        ang_diff > 0.0 ?
            std::min (max_vel_th_, std::max (min_in_place_vel_th_, ang_diff)) :
            std::max (min_vel_th_,
                      std::min (-1.0 * min_in_place_vel_th_, ang_diff));
    
    //take the acceleration limits of the robot into account
    double max_acc_vel = fabs (vel_yaw) + acc_lim_theta_ * sim_period_;
    double min_acc_vel = fabs (vel_yaw) - acc_lim_theta_ * sim_period_;
    
    v_theta_samp = sign (v_theta_samp)
        * std::min (std::max (fabs (v_theta_samp), min_acc_vel), max_acc_vel);
    
    //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
    double max_speed_to_stop = sqrt (2 * acc_lim_theta_ * fabs (ang_diff));
    
    v_theta_samp = sign (v_theta_samp)
        * std::min (max_speed_to_stop, fabs (v_theta_samp));
    
    // Re-enforce min_in_place_vel_th_.  It is more important than the acceleration limits.
    v_theta_samp =
        v_theta_samp > 0.0 ?
            std::min (max_vel_th_,
                      std::max (min_in_place_vel_th_, v_theta_samp)) :
            std::max (min_vel_th_,
                      std::min (-1.0 * min_in_place_vel_th_, v_theta_samp));
    
    //we still want to lay down the footprint of the robot and check if the action is legal
    bool valid_cmd = tc_->checkTrajectory (global_pose.getOrigin ().getX (),
                                           global_pose.getOrigin ().getY (),
                                           yaw, robot_vel.getOrigin ().getX (),
                                           robot_vel.getOrigin ().getY (),
                                           vel_yaw, 0.0, 0.0, v_theta_samp);
    
    NS_NaviCommon::console.debug (
        "Moving to desired goal orientation, th cmd: %.2f, valid_cmd: %d",
        v_theta_samp, valid_cmd);
    
    if (valid_cmd)
    {
      cmd_vel.angular.z = v_theta_samp;
      return true;
    }
    
    cmd_vel.angular.z = 0.0;
    return false;
    
  }
  
  bool
  TrajectoryLocalPlanner::setPlan (
      const std::vector<NS_DataType::PoseStamped>& orig_global_plan)
  {
    if (!isInitialized ())
    {
      NS_NaviCommon::console.error (
          "This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    
    //reset the global plan
    global_plan_.clear ();
    global_plan_ = orig_global_plan;
    
    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    xy_tolerance_latch_ = false;
    //reset the at goal flag
    reached_goal_ = false;
    return true;
  }
  
  bool
  TrajectoryLocalPlanner::computeVelocityCommands (NS_DataType::Twist& cmd_vel)
  {
    if (!isInitialized ())
    {
      NS_NaviCommon::console.error (
          "This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    
    std::vector<NS_DataType::PoseStamped> local_plan;
    NS_Transform::Stamped<NS_Transform::Pose> global_pose;
    if (!costmap->getRobotPose (global_pose))
    {
      return false;
    }
    
    std::vector<NS_DataType::PoseStamped> transformed_plan;
    //get the global plan in our frame
    if (!transformGlobalPlan (service, global_plan_, global_pose, *costmap_,
                              transformed_plan))
    {
      NS_NaviCommon::console.warning (
          "Could not transform the global plan to the frame of the controller");
      return false;
    }
    
    //now we'll prune the plan based on the position of the robot
    if (prune_plan_)
      prunePlan (global_pose, transformed_plan, global_plan_);
    
    NS_Transform::Stamped<NS_Transform::Pose> drive_cmds;
    
    NS_Transform::Stamped<NS_Transform::Pose> robot_vel;
    odom_helper_->getRobotVel (robot_vel);
    
    //if the global plan passed in is empty... we won't do anything
    if (transformed_plan.empty ())
      return false;
    
    NS_Transform::Stamped<NS_Transform::Pose> goal_point;
    NS_Transform::poseStampedMsgToTF (transformed_plan.back (), goal_point);
    //we assume the global goal is the last point in the global plan
    double goal_x = goal_point.getOrigin ().getX ();
    double goal_y = goal_point.getOrigin ().getY ();
    
    double yaw = NS_Transform::getYaw (goal_point.getRotation ());
    
    double goal_th = yaw;
    
    //check to see if we've reached the goal position
    if (xy_tolerance_latch_
        || (getGoalPositionDistance (global_pose, goal_x, goal_y)
            <= xy_goal_tolerance_))
    {
      
      //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
      //just rotate in place
      if (latch_xy_goal_tolerance_)
      {
        xy_tolerance_latch_ = true;
      }
      
      double angle = getGoalOrientationAngleDifference (global_pose, goal_th);
      //check to see if the goal orientation has been reached
      if (fabs (angle) <= yaw_goal_tolerance_)
      {
        //set the velocity command to zero
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        rotating_to_goal_ = false;
        xy_tolerance_latch_ = false;
        reached_goal_ = true;
      }
      else
      {
        //we need to call the next two lines to make sure that the trajectory
        //planner updates its path distance and goal distance grids
        tc_->updatePlan (transformed_plan);
        Trajectory path = tc_->findBestPath (global_pose, robot_vel,
                                             drive_cmds);
        
        //copy over the odometry information
        NS_DataType::Odometry base_odom;
        odom_helper_->getOdom (base_odom);
        
        //if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
        if (!rotating_to_goal_
            && !NS_Planner::stopped (base_odom, rot_stopped_velocity_,
                                     trans_stopped_velocity_))
        {
          if (!stopWithAccLimits (global_pose, robot_vel, cmd_vel))
          {
            return false;
          }
        }
        //if we're stopped... then we want to rotate to goal
        else
        {
          //set this so that we know its OK to be moving
          rotating_to_goal_ = true;
          if (!rotateToGoal (global_pose, robot_vel, goal_th, cmd_vel))
          {
            return false;
          }
        }
      }
      
      //we don't actually want to run the controller when we're just rotating to goal
      return true;
    }
    
    tc_->updatePlan (transformed_plan);
    
    //compute what trajectory to drive along
    Trajectory path = tc_->findBestPath (global_pose, robot_vel, drive_cmds);
    
    /* For timing uncomment
     gettimeofday(&end, NULL);
     start_t = start.tv_sec + double(start.tv_usec) / 1e6;
     end_t = end.tv_sec + double(end.tv_usec) / 1e6;
     t_diff = end_t - start_t;
     ROS_INFO("Cycle time: %.9f", t_diff);
     */

    //pass along drive commands
    cmd_vel.linear.x = drive_cmds.getOrigin ().getX ();
    cmd_vel.linear.y = drive_cmds.getOrigin ().getY ();
    cmd_vel.angular.z = NS_Transform::getYaw (drive_cmds.getRotation ());
    
    //if we cannot move... tell someone
    if (path.cost_ < 0)
    {
      NS_NaviCommon::console.debug (
          "The rollout planner failed to find a valid plan. This means that the footprint of the robot was in collision for all simulated trajectories.");
      local_plan.clear ();
      return false;
    }
    
    NS_NaviCommon::console.debug (
        "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.",
        cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
    
    // Fill out the local plan
    for (unsigned int i = 0; i < path.getPointsSize (); ++i)
    {
      double p_x, p_y, p_th;
      path.getPoint (i, p_x, p_y, p_th);
      NS_Transform::Stamped<NS_Transform::Pose> p = NS_Transform::Stamped<
          NS_Transform::Pose> (
          NS_Transform::Pose (NS_Transform::createQuaternionFromYaw (p_th),
                              NS_Transform::Point (p_x, p_y, 0.0)),
          NS_NaviCommon::Time::now (), "");
      NS_DataType::PoseStamped pose;
      NS_Transform::poseStampedTFToMsg (p, pose);
      local_plan.push_back (pose);
    }
    
    return true;
  }
  
  bool
  TrajectoryLocalPlanner::checkTrajectory (double vx_samp, double vy_samp,
                                           double vtheta_samp, bool update_map)
  {
    NS_Transform::Stamped<NS_Transform::Pose> global_pose;
    if (costmap->getRobotPose (global_pose))
    {
      if (update_map)
      {
        //we need to give the planne some sort of global plan, since we're only checking for legality
        //we'll just give the robots current position
        std::vector<NS_DataType::PoseStamped> plan;
        NS_DataType::PoseStamped pose_msg;
        NS_Transform::poseStampedTFToMsg (global_pose, pose_msg);
        plan.push_back (pose_msg);
        tc_->updatePlan (plan, true);
      }
      
      //copy over the odometry information
      NS_DataType::Odometry base_odom;
      {
        boost::recursive_mutex::scoped_lock lock (odom_lock_);
        base_odom = base_odom_;
      }
      
      return tc_->checkTrajectory (
          global_pose.getOrigin ().x (), global_pose.getOrigin ().y (),
          NS_Transform::getYaw (global_pose.getRotation ()),
          base_odom.twist.linear.x, base_odom.twist.linear.y,
          base_odom.twist.angular.z, vx_samp, vy_samp, vtheta_samp);
      
    }
    NS_NaviCommon::console.warning (
        "Failed to get the pose of the robot. No trajectories will pass as legal in this case.");
    return false;
  }
  
  double
  TrajectoryLocalPlanner::scoreTrajectory (double vx_samp, double vy_samp,
                                           double vtheta_samp, bool update_map)
  {
    // Copy of checkTrajectory that returns a score instead of True / False
    NS_Transform::Stamped<NS_Transform::Pose> global_pose;
    if (costmap->getRobotPose (global_pose))
    {
      if (update_map)
      {
        //we need to give the planne some sort of global plan, since we're only checking for legality
        //we'll just give the robots current position
        std::vector<NS_DataType::PoseStamped> plan;
        NS_DataType::PoseStamped pose_msg;
        NS_Transform::poseStampedTFToMsg (global_pose, pose_msg);
        plan.push_back (pose_msg);
        tc_->updatePlan (plan, true);
      }
      
      //copy over the odometry information
      NS_DataType::Odometry base_odom;
      {
        boost::recursive_mutex::scoped_lock lock (odom_lock_);
        base_odom = base_odom_;
      }
      
      return tc_->scoreTrajectory (
          global_pose.getOrigin ().x (), global_pose.getOrigin ().y (),
          NS_Transform::getYaw (global_pose.getRotation ()),
          base_odom.twist.linear.x, base_odom.twist.linear.y,
          base_odom.twist.angular.z, vx_samp, vy_samp, vtheta_samp);
      
    }
    NS_NaviCommon::console.warning (
        "Failed to get the pose of the robot. No trajectories will pass as legal in this case.");
    return -1.0;
  }
  
  bool
  TrajectoryLocalPlanner::isGoalReached ()
  {
    if (!isInitialized ())
    {
      NS_NaviCommon::console.error (
          "This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    //return flag set in controller
    return reached_goal_;
  }
}
;
