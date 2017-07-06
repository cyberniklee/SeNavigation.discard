#include "DwaLocalPlanner.h"
#include <Eigen/Core>
#include <cmath>

#include <Console/Console.h>

#include "../TrajectoryLocalPlanner/Algorithm/GoalFunctions.h"
#include <DataSet/DataType/Path.h>
#include <Parameter/Parameter.h>

namespace NS_Planner {

  DwaLocalPlanner::DwaLocalPlanner() : initialized_(false),
      odom_helper_(service), setup_(false) {

  }

  DwaLocalPlanner::~DwaLocalPlanner(){
      //make sure to clean things up
    if(latchedStopRotateController_)
      delete latchedStopRotateController_;
  }

  void DwaLocalPlanner::onInitialize() {
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

      bool latch_xy_goal_tolerance;

      NS_NaviCommon::Parameter parameter;

      costmap->getRobotPose(current_pose_);

      // make sure to update the costmap we'll use for this cycle
      NS_CostMap::Costmap2D* costmap2d = costmap->getCostmap();

      planner_util_.initialize(service, costmap2d);

      //todo:fix these parameters

      parameter.loadConfigurationFile("trajectory_local_planner.xml");
      if(parameter.getParameter("sum_scores", 0) == 1)
      {
        sum_scores = true;
      }else{
        sum_scores = false;
      }

      cheat_factor = parameter.getParameter("cheat_factor", 1.0f);
      sim_time = parameter.getParameter("sim_time", 1.7f);
      sim_granularity = parameter.getParameter("sim_granularity", 0.025f);
      angular_sim_granularity = parameter.getParameter("angular_sim_granularity", 0.025f);
      pdist_scale = parameter.getParameter("pdist_scale", 32.0f);
      gdist_scale = parameter.getParameter("gdist_scale", 24.0f);
      occdist_scale = parameter.getParameter("occdist_scale", 0.01f);
      stop_time_buffer = parameter.getParameter("stop_time_buffer", 0.2f);
      oscillation_reset_dist = parameter.getParameter("oscillation_reset_dist", 0.05f);
      oscillation_reset_angle = parameter.getParameter("oscillation_reset_angle", 0.2f);
      forward_point_dist = parameter.getParameter("forward_point_dist", 0.325f);
      max_trans_vel = parameter.getParameter("max_trans_vel", 0.55f);
      scaling_speed = parameter.getParameter("scaling_speed", 0.25f);
      max_scaling_factor = parameter.getParameter("max_scaling_factor", 0.2f);
      vx_samples = parameter.getParameter("vx_samples", 3);
      vy_samples = parameter.getParameter("vy_samples", 10);
      vth_samples = parameter.getParameter("vth_samples", 20);

      if(parameter.getParameter("dwa", 1) == 1)
      {
        dwa = true;
      }else{
        dwa = false;
      }

      sim_period = parameter.getParameter("vth_samples", 0.01f);

      //create the actual planner that we'll use.. it'll configure itself from the parameter server
      dp_ = boost::shared_ptr<DWAPlanner>(new DWAPlanner(&planner_util_, sum_scores, cheat_factor,
                                                         sim_time, sim_granularity, angular_sim_granularity,
                                                         pdist_scale, gdist_scale, occdist_scale,
                                                         stop_time_buffer, oscillation_reset_dist, oscillation_reset_angle,
                                                         forward_point_dist, max_trans_vel, scaling_speed, max_scaling_factor,
                                                         vx_samples, vy_samples, vth_samples,
                                                         dwa, sim_period));
      
      if(parameter.getParameter("latch_xy_goal_tolerance", 0) == 1)
      {
        latch_xy_goal_tolerance = true;
      }else{
        latch_xy_goal_tolerance = false;
      }

      latchedStopRotateController_ = new LatchedStopRotateController(latch_xy_goal_tolerance);

      initialized_ = true;
    }
    else{
      NS_NaviCommon::console.warning("This planner has already been initialized, doing nothing.");
    }
  }
  
  bool DwaLocalPlanner::setPlan(const std::vector<NS_DataType::PoseStamped>& orig_global_plan) {
    if (! isInitialized()) {
      NS_NaviCommon::console.error("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    latchedStopRotateController_->resetLatching();

    NS_NaviCommon::console.message("Got new plan");
    return dp_->setPlan(orig_global_plan);
  }

  bool DwaLocalPlanner::isGoalReached() {
    if (! isInitialized()) {
      NS_NaviCommon::console.error("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    if ( ! costmap->getRobotPose(current_pose_)) {
      NS_NaviCommon::console.error("Could not get robot pose");
      return false;
    }

    if(latchedStopRotateController_->isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
      NS_NaviCommon::console.message("Goal reached");
      return true;
    } else {
      return false;
    }
  }

  bool DwaLocalPlanner::dwaComputeVelocityCommands(NS_Transform::Stamped<NS_Transform::Pose> &global_pose, NS_DataType::Twist& cmd_vel) {
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




  bool DwaLocalPlanner::computeVelocityCommands(NS_DataType::Twist& cmd_vel) {
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

    if (latchedStopRotateController_->isPositionReached(&planner_util_, current_pose_)) {
      //publish an empty plan because we've reached our goal position
      std::vector<NS_DataType::PoseStamped> local_plan;
      std::vector<NS_DataType::PoseStamped> transformed_plan;
      NS_Planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
      return latchedStopRotateController_->computeVelocityCommandsStopRotate(
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
