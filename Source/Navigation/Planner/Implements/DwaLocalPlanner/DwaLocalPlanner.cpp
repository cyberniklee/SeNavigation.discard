/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

#include "DwaLocalPlanner.h"
#include <Eigen/Core>
#include <cmath>

#include <Console/Console.h>

#include "../TrajectoryLocalPlanner/Algorithm/GoalFunctions.h"
#include <DataSet/DataType/Path.h>

namespace NS_Planner {

  DWAPlannerROS::DWAPlannerROS() : initialized_(false),
      odom_helper_(service), setup_(false) {

  }

  void DWAPlannerROS::onInitialize() {
    if (! isInitialized())
    {
      bool sum_scores;
      double cheat_factor;
      double sim_time;
      double sim_granularity;
      double angular_sim_granularity;
      double pdist_scale;
      double gdist_scale;
      double occdist_scale;
      double stop_time_buffer;
      double oscillation_reset_dist;
      double oscillation_reset_angle;
      double forward_point_dist;
      double max_trans_vel;
      double scaling_speed;
      double max_scaling_factor;
      int vx_samples;
      int vy_samples;
      int vth_samples;
      bool dwa;
      double sim_period;

      costmap->getRobotPose(current_pose_);

      // make sure to update the costmap we'll use for this cycle
      NS_CostMap::Costmap2D* costmap2d = costmap->getCostmap();

      planner_util_.initialize(service, costmap2d);

      //todo:fix these parameters

      //create the actual planner that we'll use.. it'll configure itself from the parameter server
      dp_ = boost::shared_ptr<DWAPlanner>(new DWAPlanner(&planner_util_, sum_scores, cheat_factor,
                                                         sim_time, sim_granularity, angular_sim_granularity,
                                                         pdist_scale, gdist_scale, occdist_scale,
                                                         stop_time_buffer, oscillation_reset_dist, oscillation_reset_angle,
                                                         forward_point_dist, max_trans_vel, scaling_speed, max_scaling_factor,
                                                         vx_samples, vy_samples, vth_samples,
                                                         dwa, sim_period));
      
      initialized_ = true;
    }
    else{
      NS_NaviCommon::console.warning("This planner has already been initialized, doing nothing.");
    }
  }
  
  bool DWAPlannerROS::setPlan(const std::vector<NS_DataType::PoseStamped>& orig_global_plan) {
    if (! isInitialized()) {
      NS_NaviCommon::console.error("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    latchedStopRotateController_.resetLatching();

    NS_NaviCommon::console.message("Got new plan");
    return dp_->setPlan(orig_global_plan);
  }

  bool DWAPlannerROS::isGoalReached() {
    if (! isInitialized()) {
      NS_NaviCommon::console.error("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    if ( ! costmap->getRobotPose(current_pose_)) {
      NS_NaviCommon::console.error("Could not get robot pose");
      return false;
    }

    if(latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
      NS_NaviCommon::console.message("Goal reached");
      return true;
    } else {
      return false;
    }
  }

  DWAPlannerROS::~DWAPlannerROS(){
    //make sure to clean things up

  }



  bool DWAPlannerROS::dwaComputeVelocityCommands(NS_Transform::Stamped<NS_Transform::Pose> &global_pose, NS_DataType::Twist& cmd_vel) {
    // dynamic window sampling approach to get useful velocity commands
    if(! isInitialized()){
      NS_NaviCommon::console.error("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    NS_Transform::Stamped<NS_Transform::Pose> robot_vel;
    odom_helper_.getRobotVel(robot_vel);

    /* For timing uncomment
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    */

    //compute what trajectory to drive along
    NS_Transform::Stamped<NS_Transform::Pose> drive_cmds;
    
    // call with updated footprint
    NS_Planner::Trajectory path = dp_->findBestPath(global_pose, robot_vel, drive_cmds, costmap->getRobotFootprint());
    //ROS_ERROR("Best: %.2f, %.2f, %.2f, %.2f", path.xv_, path.yv_, path.thetav_, path.cost_);

    /* For timing uncomment
    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    ROS_INFO("Cycle time: %.9f", t_diff);
    */

    //pass along drive commands
    cmd_vel.linear.x = drive_cmds.getOrigin().getX();
    cmd_vel.linear.y = drive_cmds.getOrigin().getY();
    cmd_vel.angular.z = NS_Transform::getYaw(drive_cmds.getRotation());

    //if we cannot move... tell someone
    std::vector<NS_DataType::PoseStamped> local_plan;
    if(path.cost_ < 0) {
      NS_NaviCommon::console.debug("The dwa local planner failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot.");
      local_plan.clear();
      return false;
    }

    NS_NaviCommon::console.debug("A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.",
                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    // Fill out the local plan
    for(unsigned int i = 0; i < path.getPointsSize(); ++i) {
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);

      NS_Transform::Stamped<NS_Transform::Pose> p =
          NS_Transform::Stamped<NS_Transform::Pose>(NS_Transform::Pose(
              NS_Transform::createQuaternionFromYaw(p_th),
              NS_Transform::Point(p_x, p_y, 0.0)),
                      NS_NaviCommon::Time::now(),"");
      NS_DataType::PoseStamped pose;
      NS_Transform::poseStampedTFToMsg(p, pose);
      local_plan.push_back(pose);
    }

    return true;
  }




  bool DWAPlannerROS::computeVelocityCommands(NS_DataType::Twist& cmd_vel) {
    // dispatches to either dwa sampling control or stop and rotate control, depending on whether we have been close enough to goal
    if ( ! costmap->getRobotPose(current_pose_)) {
      NS_NaviCommon::console.error("Could not get robot pose");
      return false;
    }
    std::vector<NS_DataType::PoseStamped> transformed_plan;
    if ( ! planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
      NS_NaviCommon::console.error("Could not get local plan");
      return false;
    }

    //if the global plan passed in is empty... we won't do anything
    if(transformed_plan.empty()) {
      NS_NaviCommon::console.warning("Received an empty transformed plan.");
      return false;
    }
    NS_NaviCommon::console.debug("Received a transformed plan with %zu points.", transformed_plan.size());

    // update plan in dwa_planner even if we just stop and rotate, to allow checkTrajectory
    dp_->updatePlanAndLocalCosts(current_pose_, transformed_plan);

    if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_)) {
      //publish an empty plan because we've reached our goal position
      std::vector<NS_DataType::PoseStamped> local_plan;
      std::vector<NS_DataType::PoseStamped> transformed_plan;
      NS_Planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
      return latchedStopRotateController_.computeVelocityCommandsStopRotate(
          cmd_vel,
          limits.getAccLimits(),
          dp_->getSimPeriod(),
          &planner_util_,
          odom_helper_,
          current_pose_,
          boost::bind(&DWAPlanner::checkTrajectory, dp_, _1, _2, _3));
    } else {
      bool isOk = dwaComputeVelocityCommands(current_pose_, cmd_vel);
      if (!isOk) {
        NS_NaviCommon::console.warning("DWA planner failed to produce path.");
      }
      return isOk;
    }
  }


};
