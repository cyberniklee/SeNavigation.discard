/*
 * latched_stop_rotate_controller.cpp
 *
 *  Created on: Apr 16, 2012
 *      Author: tkruse
 */

#include "LatchedStopRotateController.h"

#include <cmath>

#include <Eigen/Core>

#include <Geometry/Angles.h>
#include <DataSet/DataType/Odometry.h>

#include "../../TrajectoryLocalPlanner/Algorithm/GoalFunctions.h"
#include "../../TrajectoryLocalPlanner/Algorithm/LocalPlannerLimits.h"

#include <Console/Console.h>

namespace NS_Planner {

LatchedStopRotateController::LatchedStopRotateController(bool latch_xy_goal_tolerance)
{
  latch_xy_goal_tolerance_ = latch_xy_goal_tolerance;

  rotating_to_goal_ = false;
}

LatchedStopRotateController::~LatchedStopRotateController() {}


/**
 * returns true if we have passed the goal position.
 * Meaning we might have overshot on the position beyond tolerance, yet still return true.
 * Also goal orientation might not yet be true
 */
bool LatchedStopRotateController::isPositionReached(LocalPlannerUtil* planner_util,
                                                    NS_Transform::Stamped<NS_Transform::Pose> global_pose) {
  double xy_goal_tolerance = planner_util->getCurrentLimits().xy_goal_tolerance;

  //we assume the global goal is the last point in the global plan
  NS_Transform::Stamped<NS_Transform::Pose> goal_pose;
  if ( ! planner_util->getGoal(goal_pose)) {
    return false;
  }

  double goal_x = goal_pose.getOrigin().getX();
  double goal_y = goal_pose.getOrigin().getY();

  //check to see if we've reached the goal position
  if ((latch_xy_goal_tolerance_ && xy_tolerance_latch_) ||
      NS_Planner::getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance) {
    xy_tolerance_latch_ = true;
    return true;
  }
  return false;
}


/**
 * returns true if we have passed the goal position and have reached goal orientation.
 * Meaning we might have overshot on the position beyond tolerance, yet still return true.
 */
bool LatchedStopRotateController::isGoalReached(LocalPlannerUtil* planner_util,
    OdometryHelper& odom_helper,
    NS_Transform::Stamped<NS_Transform::Pose> global_pose) {
  double xy_goal_tolerance = planner_util->getCurrentLimits().xy_goal_tolerance;
  double rot_stopped_vel = planner_util->getCurrentLimits().rot_stopped_vel;
  double trans_stopped_vel = planner_util->getCurrentLimits().trans_stopped_vel;

  //copy over the odometry information
  NS_DataType::Odometry base_odom;
  odom_helper.getOdom(base_odom);

  //we assume the global goal is the last point in the global plan
  NS_Transform::Stamped<NS_Transform::Pose> goal_pose;
  if ( ! planner_util->getGoal(goal_pose)) {
    return false;
  }

  double goal_x = goal_pose.getOrigin().getX();
  double goal_y = goal_pose.getOrigin().getY();

  NS_Planner::LocalPlannerLimits limits = planner_util->getCurrentLimits();

  //check to see if we've reached the goal position
  if ((latch_xy_goal_tolerance_ && xy_tolerance_latch_) ||
      NS_Planner::getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance) {
    //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
    //just rotate in place
    if (latch_xy_goal_tolerance_ && ! xy_tolerance_latch_) {
      NS_NaviCommon::console.debug("Goal position reached (check), stopping and turning in place");
      xy_tolerance_latch_ = true;
    }
    double goal_th = NS_Transform::getYaw(goal_pose.getRotation());
    double angle = NS_Planner::getGoalOrientationAngleDifference(global_pose, goal_th);
    //check to see if the goal orientation has been reached
    if (fabs(angle) <= limits.yaw_goal_tolerance) {
      //make sure that we're actually stopped before returning success
      if (NS_Planner::stopped(base_odom, rot_stopped_vel, trans_stopped_vel)) {
        return true;
      }
    }
  }
  return false;
}

bool LatchedStopRotateController::stopWithAccLimits(const NS_Transform::Stamped<NS_Transform::Pose>& global_pose,
    const NS_Transform::Stamped<NS_Transform::Pose>& robot_vel,
    NS_DataType::Twist& cmd_vel,
    Eigen::Vector3f acc_lim,
    double sim_period,
    boost::function<bool (Eigen::Vector3f pos,
                          Eigen::Vector3f vel,
                          Eigen::Vector3f vel_samples)> obstacle_check) {

  //slow down with the maximum possible acceleration... we should really use the frequency that we're running at to determine what is feasible
  //but we'll use a tenth of a second to be consistent with the implementation of the local planner.
  double vx = sign(robot_vel.getOrigin().x()) * std::max(0.0, (fabs(robot_vel.getOrigin().x()) - acc_lim[0] * sim_period));
  double vy = sign(robot_vel.getOrigin().y()) * std::max(0.0, (fabs(robot_vel.getOrigin().y()) - acc_lim[1] * sim_period));

  double vel_yaw = NS_Transform::getYaw(robot_vel.getRotation());
  double vth = sign(vel_yaw) * std::max(0.0, (fabs(vel_yaw) - acc_lim[2] * sim_period));

  //we do want to check whether or not the command is valid
  double yaw = NS_Transform::getYaw(global_pose.getRotation());
  bool valid_cmd = obstacle_check(Eigen::Vector3f(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), yaw),
                                  Eigen::Vector3f(robot_vel.getOrigin().getX(), robot_vel.getOrigin().getY(), vel_yaw),
                                  Eigen::Vector3f(vx, vy, vth));

  //if we have a valid command, we'll pass it on, otherwise we'll command all zeros
  if(valid_cmd){
    NS_NaviCommon::console.debug("Slowing down... using vx, vy, vth: %.2f, %.2f, %.2f", vx, vy, vth);
    cmd_vel.linear.x = vx;
    cmd_vel.linear.y = vy;
    cmd_vel.angular.z = vth;
    return true;
  }
  NS_NaviCommon::console.warning("Stopping cmd in collision");
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  return false;
}

bool LatchedStopRotateController::rotateToGoal(
    const NS_Transform::Stamped<NS_Transform::Pose>& global_pose,
    const NS_Transform::Stamped<NS_Transform::Pose>& robot_vel,
    double goal_th,
    NS_DataType::Twist& cmd_vel,
    Eigen::Vector3f acc_lim,
    double sim_period,
    NS_Planner::LocalPlannerLimits& limits,
    boost::function<bool (Eigen::Vector3f pos,
                          Eigen::Vector3f vel,
                          Eigen::Vector3f vel_samples)> obstacle_check) {
  double yaw = NS_Transform::getYaw(global_pose.getRotation());
  double vel_yaw = NS_Transform::getYaw(robot_vel.getRotation());
  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = 0;
  double ang_diff = NS_Geometry::NS_Angles::shortest_angular_distance(yaw, goal_th);

  double v_theta_samp = std::min(limits.max_rot_vel, std::max(limits.min_rot_vel, fabs(ang_diff)));

  //take the acceleration limits of the robot into account
  double max_acc_vel = fabs(vel_yaw) + acc_lim[2] * sim_period;
  double min_acc_vel = fabs(vel_yaw) - acc_lim[2] * sim_period;

  v_theta_samp = std::min(std::max(fabs(v_theta_samp), min_acc_vel), max_acc_vel);

  //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
  double max_speed_to_stop = sqrt(2 * acc_lim[2] * fabs(ang_diff));
  v_theta_samp = std::min(max_speed_to_stop, fabs(v_theta_samp));

  v_theta_samp = std::min(limits.max_rot_vel, std::max(limits.min_rot_vel, v_theta_samp));

  if (ang_diff < 0) {
    v_theta_samp = - v_theta_samp;
  }

  //we still want to lay down the footprint of the robot and check if the action is legal
  bool valid_cmd = obstacle_check(Eigen::Vector3f(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), yaw),
      Eigen::Vector3f(robot_vel.getOrigin().getX(), robot_vel.getOrigin().getY(), vel_yaw),
      Eigen::Vector3f( 0.0, 0.0, v_theta_samp));

  if (valid_cmd) {
    NS_NaviCommon::console.debug("Moving to desired goal orientation, th cmd: %.2f, valid_cmd: %d", v_theta_samp, valid_cmd);
    cmd_vel.angular.z = v_theta_samp;
    return true;
  }
  NS_NaviCommon::console.warning("Rotation cmd in collision");
  cmd_vel.angular.z = 0.0;
  return false;

}

bool LatchedStopRotateController::computeVelocityCommandsStopRotate(NS_DataType::Twist& cmd_vel,
    Eigen::Vector3f acc_lim,
    double sim_period,
    LocalPlannerUtil* planner_util,
    OdometryHelper& odom_helper_,
    NS_Transform::Stamped<NS_Transform::Pose> global_pose,
    boost::function<bool (Eigen::Vector3f pos,
                          Eigen::Vector3f vel,
                          Eigen::Vector3f vel_samples)> obstacle_check) {
  //we assume the global goal is the last point in the global plan
  NS_Transform::Stamped<NS_Transform::Pose> goal_pose;
  if ( ! planner_util->getGoal(goal_pose)) {
    NS_NaviCommon::console.error("Could not get goal pose");
    return false;
  }

  NS_Planner::LocalPlannerLimits limits = planner_util->getCurrentLimits();

  //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
  //just rotate in place
  if (latch_xy_goal_tolerance_ && ! xy_tolerance_latch_ ) {
    NS_NaviCommon::console.message("Goal position reached, stopping and turning in place");
    xy_tolerance_latch_ = true;
  }
  //check to see if the goal orientation has been reached
  double goal_th = NS_Transform::getYaw(goal_pose.getRotation());
  double angle = NS_Planner::getGoalOrientationAngleDifference(global_pose, goal_th);
  if (fabs(angle) <= limits.yaw_goal_tolerance) {
    //set the velocity command to zero
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    rotating_to_goal_ = false;
  } else {
    NS_NaviCommon::console.debug("Angle: %f Tolerance: %f", angle, limits.yaw_goal_tolerance);
    NS_Transform::Stamped<NS_Transform::Pose> robot_vel;
    odom_helper_.getRobotVel(robot_vel);
    NS_DataType::Odometry base_odom;
    odom_helper_.getOdom(base_odom);

    //if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
    if ( ! rotating_to_goal_ && !NS_Planner::stopped(base_odom, limits.rot_stopped_vel, limits.trans_stopped_vel)) {
      if ( ! stopWithAccLimits(
          global_pose,
          robot_vel,
          cmd_vel,
          acc_lim,
          sim_period,
          obstacle_check)) {
        NS_NaviCommon::console.message("Error when stopping.");
        return false;
      }
      NS_NaviCommon::console.debug("Stopping...");
    }
    //if we're stopped... then we want to rotate to goal
    else {
      //set this so that we know its OK to be moving
      rotating_to_goal_ = true;
      if ( ! rotateToGoal(
          global_pose,
          robot_vel,
          goal_th,
          cmd_vel,
          acc_lim,
          sim_period,
          limits,
          obstacle_check)) {
        NS_NaviCommon::console.message("Error when rotating.");
        return false;
      }
      NS_NaviCommon::console.debug("Rotating...");
    }
  }

  return true;

}


} /* namespace base_local_planner */
