#ifndef _DWA_LOCAL_PLANNER_LATCHED_STOP_ROTATE_CONTROLLER_H_
#define _DWA_LOCAL_PLANNER_LATCHED_STOP_ROTATE_CONTROLLER_H_

#include <string>

#include <Eigen/Core>

#include <Transform/DataTypes.h>

#include "LocalPlannerUtil.h"
#include "../../TrajectoryLocalPlanner/Algorithm/LocalPlannerLimits.h"
#include "../../TrajectoryLocalPlanner/Algorithm/OdometryHelper.h"

namespace NS_Planner
{
  
  class LatchedStopRotateController
  {
  public:
    LatchedStopRotateController (bool latch_xy_goal_tolerance);
    virtual
    ~LatchedStopRotateController ();

    bool
    isPositionReached (LocalPlannerUtil* planner_util,
                       NS_Transform::Stamped<NS_Transform::Pose> global_pose);

    bool
    isGoalReached (LocalPlannerUtil* planner_util, OdometryHelper& odom_helper,
                   NS_Transform::Stamped<NS_Transform::Pose> global_pose);

    void
    resetLatching ()
    {
      xy_tolerance_latch_ = false;
    }
    
    /**
     * @brief Stop the robot taking into account acceleration limits
     * @param  global_pose The pose of the robot in the global frame
     * @param  robot_vel The velocity of the robot
     * @param  cmd_vel The velocity commands to be filled
     * @return  True if a valid trajectory was found, false otherwise
     */
    bool
    stopWithAccLimits (
        const NS_Transform::Stamped<NS_Transform::Pose>& global_pose,
        const NS_Transform::Stamped<NS_Transform::Pose>& robot_vel,
        NS_DataType::Twist& cmd_vel,
        Eigen::Vector3f acc_lim,
        double sim_period,
        boost::function<bool
        (Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f vel_samples)> obstacle_check);

    /**
     * @brief Once a goal position is reached... rotate to the goal orientation
     * @param  global_pose The pose of the robot in the global frame
     * @param  robot_vel The velocity of the robot
     * @param  goal_th The desired th value for the goal
     * @param  cmd_vel The velocity commands to be filled
     * @return  True if a valid trajectory was found, false otherwise
     */
    bool
    rotateToGoal (
        const NS_Transform::Stamped<NS_Transform::Pose>& global_pose,
        const NS_Transform::Stamped<NS_Transform::Pose>& robot_vel,
        double goal_th,
        NS_DataType::Twist& cmd_vel,
        Eigen::Vector3f acc_lim,
        double sim_period,
        NS_Planner::LocalPlannerLimits& limits,
        boost::function<bool
        (Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f vel_samples)> obstacle_check);

    bool
    computeVelocityCommandsStopRotate (
        NS_DataType::Twist& cmd_vel,
        Eigen::Vector3f acc_lim,
        double sim_period,
        LocalPlannerUtil* planner_util,
        OdometryHelper& odom_helper,
        NS_Transform::Stamped<NS_Transform::Pose> global_pose,
        boost::function<bool
        (Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f vel_samples)> obstacle_check);

  private:
    inline double
    sign (double x)
    {
      return x < 0.0 ? -1.0 : 1.0;
    }
    
    // whether to latch at all, and whether in this turn we have already been in goal area
    bool latch_xy_goal_tolerance_, xy_tolerance_latch_;
    bool rotating_to_goal_;
  };

} /* namespace base_local_planner */
#endif /* LATCHED_STOP_ROTATE_CONTROLLER_H_ */
