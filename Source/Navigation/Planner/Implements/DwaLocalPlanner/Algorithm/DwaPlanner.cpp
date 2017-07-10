#include "DwaPlanner.h"
#include "../../TrajectoryLocalPlanner/Algorithm/GoalFunctions.h"
#include "../../../../CostMap/CostMap2D/CostValues.h"
#include "MapGridCostFunction.h"
#include <cmath>

//for computing path distance
#include <queue>

#include <Geometry/Angles.h>
#include <Console/Console.h>

namespace NS_Planner
{
  
  DWAPlanner::DWAPlanner (NS_Planner::LocalPlannerUtil *planner_util,
                          bool sum_scores, double cheat_factor, double sim_time,
                          double sim_granularity,
                          double angular_sim_granularity, double pdist_scale,
                          double gdist_scale, double occdist_scale,
                          double stop_time_buffer,
                          double oscillation_reset_dist,
                          double oscillation_reset_angle,
                          double forward_point_dist, double max_trans_vel,
                          double scaling_speed, double max_scaling_factor,
                          int vx_samples, int vy_samples, int vth_samples,
                          bool dwa, double sim_period)
      : planner_util_ (planner_util),
          obstacle_costs_ (planner_util->getCostmap ()),
          path_costs_ (planner_util->getCostmap ()),
          goal_costs_ (planner_util->getCostmap (), 0.0, 0.0, true),
          goal_front_costs_ (planner_util->getCostmap (), 0.0, 0.0, true),
          alignment_costs_ (planner_util->getCostmap ())
  {
    
    goal_front_costs_.setStopOnFailure (false);
    alignment_costs_.setStopOnFailure (false);
    
    sim_period_ = sim_period;
    
    oscillation_costs_.resetOscillationFlags ();
    
    obstacle_costs_.setSumScores (sum_scores);
    
    // set up all the cost functions that will be applied in order
    // (any function returning negative values will abort scoring, so the order can improve performance)
    std::vector<NS_Planner::TrajectoryCostFunction*> critics;
    critics.push_back (&oscillation_costs_); // discards oscillating motions (assisgns cost -1)
    critics.push_back (&obstacle_costs_); // discards trajectories that move into obstacles
    critics.push_back (&goal_front_costs_); // prefers trajectories that make the nose go towards (local) nose goal
    critics.push_back (&alignment_costs_); // prefers trajectories that keep the robot nose on nose path
    critics.push_back (&path_costs_); // prefers trajectories on global path
    critics.push_back (&goal_costs_); // prefers trajectories that go towards (local) goal, based on wave propagation
        
    // trajectory generators
    std::vector<NS_Planner::TrajectorySampleGenerator*> generator_list;
    generator_list.push_back (&generator_);
    
    scored_sampling_planner_ = NS_Planner::SimpleScoredSamplingPlanner (
        generator_list, critics);
    
    cheat_factor_ = cheat_factor;
    
    generator_.setParameters (sim_time, sim_granularity,
                              angular_sim_granularity, dwa, sim_period_);
    
    double resolution = planner_util_->getCostmap ()->getResolution ();
    pdist_scale_ = pdist_scale;
    // pdistscale used for both path and alignment, set  forward_point_distance to zero to discard alignment
    path_costs_.setScale (resolution * pdist_scale_ * 0.5);
    alignment_costs_.setScale (resolution * pdist_scale_ * 0.5);
    
    gdist_scale_ = gdist_scale;
    goal_costs_.setScale (resolution * gdist_scale_ * 0.5);
    goal_front_costs_.setScale (resolution * gdist_scale_ * 0.5);
    
    occdist_scale_ = occdist_scale;
    obstacle_costs_.setScale (resolution * occdist_scale_);
    
    stop_time_buffer_ = stop_time_buffer;
    oscillation_costs_.setOscillationResetDist (oscillation_reset_dist,
                                                oscillation_reset_angle);
    forward_point_distance_ = forward_point_dist;
    goal_front_costs_.setXShift (forward_point_distance_);
    alignment_costs_.setXShift (forward_point_distance_);
    
    // obstacle costs can vary due to scaling footprint feature
    obstacle_costs_.setParams (max_trans_vel, max_scaling_factor,
                               scaling_speed);
    
    int vx_samp_, vy_samp_, vth_samp_;
    vx_samp_ = vx_samples;
    vy_samp_ = vy_samples;
    vth_samp_ = vth_samples;
    
    if (vx_samp_ <= 0)
    {
      vx_samp_ = 1;
    }
    
    if (vy_samp_ <= 0)
    {
      vy_samp_ = 1;
    }
    
    if (vth_samp_ <= 0)
    {
      vth_samp_ = 1;
    }
    
    vsamples_[0] = vx_samp_;
    vsamples_[1] = vy_samp_;
    vsamples_[2] = vth_samp_;
  }
  
  // used for visualization only, total_costs are not really total costs
  bool
  DWAPlanner::getCellCosts (int cx, int cy, float &path_cost, float &goal_cost,
                            float &occ_cost, float &total_cost)
  {
    
    path_cost = path_costs_.getCellCosts (cx, cy);
    goal_cost = goal_costs_.getCellCosts (cx, cy);
    occ_cost = planner_util_->getCostmap ()->getCost (cx, cy);
    if (path_cost == path_costs_.obstacleCosts ()
        || path_cost == path_costs_.unreachableCellCosts ()
        || occ_cost >= NS_CostMap::INSCRIBED_INFLATED_OBSTACLE)
    {
      return false;
    }
    
    double resolution = planner_util_->getCostmap ()->getResolution ();
    total_cost = pdist_scale_ * resolution * path_cost
        + gdist_scale_ * resolution * goal_cost + occdist_scale_ * occ_cost;
    return true;
  }
  
  bool
  DWAPlanner::setPlan (
      const std::vector<NS_DataType::PoseStamped>& orig_global_plan)
  {
    oscillation_costs_.resetOscillationFlags ();
    return planner_util_->setPlan (orig_global_plan);
  }
  
  /**
   * This function is used when other strategies are to be applied,
   * but the cost functions for obstacles are to be reused.
   */
  bool
  DWAPlanner::checkTrajectory (Eigen::Vector3f pos, Eigen::Vector3f vel,
                               Eigen::Vector3f vel_samples)
  {
    oscillation_costs_.resetOscillationFlags ();
    NS_Planner::Trajectory traj;
    NS_DataType::PoseStamped goal_pose = global_plan_.back ();
    Eigen::Vector3f goal (goal_pose.pose.position.x, goal_pose.pose.position.y,
                          NS_Transform::getYaw (goal_pose.pose.orientation));
    NS_Planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits ();
    generator_.initialise (pos, vel, goal, &limits, vsamples_);
    generator_.generateTrajectory (pos, vel, vel_samples, traj);
    double cost = scored_sampling_planner_.scoreTrajectory (traj, -1);
    //if the trajectory is a legal one... the check passes
    if (cost >= 0)
    {
      return true;
    }
    NS_NaviCommon::console.warning ("Invalid Trajectory %f, %f, %f, cost: %f",
                                    vel_samples[0], vel_samples[1],
                                    vel_samples[2], cost);
    
    //otherwise the check fails
    return false;
  }
  
  void
  DWAPlanner::updatePlanAndLocalCosts (
      NS_Transform::Stamped<NS_Transform::Pose> global_pose,
      const std::vector<NS_DataType::PoseStamped>& new_plan)
  {
    global_plan_.resize (new_plan.size ());
    for (unsigned int i = 0; i < new_plan.size (); ++i)
    {
      global_plan_[i] = new_plan[i];
    }
    
    // costs for going away from path
    path_costs_.setTargetPoses (global_plan_);
    
    // costs for not going towards the local goal as much as possible
    goal_costs_.setTargetPoses (global_plan_);
    
    // alignment costs
    NS_DataType::PoseStamped goal_pose = global_plan_.back ();
    
    Eigen::Vector3f pos (global_pose.getOrigin ().getX (),
                         global_pose.getOrigin ().getY (),
                         NS_Transform::getYaw (global_pose.getRotation ()));
    double sq_dist = (pos[0] - goal_pose.pose.position.x)
        * (pos[0] - goal_pose.pose.position.x)
        + (pos[1] - goal_pose.pose.position.y)
            * (pos[1] - goal_pose.pose.position.y);
    
    // we want the robot nose to be drawn to its final position
    // (before robot turns towards goal orientation), not the end of the
    // path for the robot center. Choosing the final position after
    // turning towards goal orientation causes instability when the
    // robot needs to make a 180 degree turn at the end
    std::vector<NS_DataType::PoseStamped> front_global_plan = global_plan_;
    double angle_to_goal = atan2 (goal_pose.pose.position.y - pos[1],
                                  goal_pose.pose.position.x - pos[0]);
    front_global_plan.back ().pose.position.x =
        front_global_plan.back ().pose.position.x
            + forward_point_distance_ * cos (angle_to_goal);
    front_global_plan.back ().pose.position.y =
        front_global_plan.back ().pose.position.y
            + forward_point_distance_ * sin (angle_to_goal);
    
    goal_front_costs_.setTargetPoses (front_global_plan);
    
    // keeping the nose on the path
    if (sq_dist
        > forward_point_distance_ * forward_point_distance_ * cheat_factor_)
    {
      double resolution = planner_util_->getCostmap ()->getResolution ();
      alignment_costs_.setScale (resolution * pdist_scale_ * 0.5);
      // costs for robot being aligned with path (nose on path, not ju
      alignment_costs_.setTargetPoses (global_plan_);
    }
    else
    {
      // once we are close to goal, trying to keep the nose close to anything destabilizes behavior.
      alignment_costs_.setScale (0.0);
    }
  }
  
  /*
   * given the current state of the robot, find a good trajectory
   */
  NS_Planner::Trajectory
  DWAPlanner::findBestPath (
      NS_Transform::Stamped<NS_Transform::Pose> global_pose,
      NS_Transform::Stamped<NS_Transform::Pose> global_vel,
      NS_Transform::Stamped<NS_Transform::Pose>& drive_velocities,
      std::vector<NS_DataType::Point> footprint_spec)
  {
    
    obstacle_costs_.setFootprint (footprint_spec);
    
    //make sure that our configuration doesn't change mid-run
    boost::mutex::scoped_lock l (configuration_mutex_);
    
    Eigen::Vector3f pos (global_pose.getOrigin ().getX (),
                         global_pose.getOrigin ().getY (),
                         NS_Transform::getYaw (global_pose.getRotation ()));
    Eigen::Vector3f vel (global_vel.getOrigin ().getX (),
                         global_vel.getOrigin ().getY (),
                         NS_Transform::getYaw (global_vel.getRotation ()));
    NS_DataType::PoseStamped goal_pose = global_plan_.back ();
    Eigen::Vector3f goal (goal_pose.pose.position.x, goal_pose.pose.position.y,
                          NS_Transform::getYaw (goal_pose.pose.orientation));
    NS_Planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits ();
    
    // prepare cost functions and generators for this run
    generator_.initialise (pos, vel, goal, &limits, vsamples_);
    
    result_traj_.cost_ = -7;
    // find best trajectory by sampling and scoring the samples
    std::vector<NS_Planner::Trajectory> all_explored;
    scored_sampling_planner_.findBestTrajectory (result_traj_, &all_explored);
    
    // debrief stateful scoring functions
    oscillation_costs_.updateOscillationFlags (
        pos, &result_traj_, planner_util_->getCurrentLimits ().min_trans_vel);
    
    //if we don't have a legal trajectory, we'll just command zero
    if (result_traj_.cost_ < 0)
    {
      drive_velocities.setIdentity ();
    }
    else
    {
      NS_Transform::Vector3 start (result_traj_.xv_, result_traj_.yv_, 0);
      drive_velocities.setOrigin (start);
      NS_Transform::Matrix3x3 matrix;
      matrix.setRotation (
          NS_Transform::createQuaternionFromYaw (result_traj_.thetav_));
      drive_velocities.setBasis (matrix);
    }
    
    return result_traj_;
  }
}
;
