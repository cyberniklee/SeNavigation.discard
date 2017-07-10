#ifndef _BASE_LOCAL_PLANNER_LIMITS_H_
#define _BASE_LOCAL_PLANNER_LIMITS_H_

#include <Eigen/Core>

namespace NS_Planner
{
  class LocalPlannerLimits
  {
  public:
    
    double max_trans_vel;
    double min_trans_vel;
    double max_vel_x;
    double min_vel_x;
    double max_vel_y;
    double min_vel_y;
    double max_rot_vel;
    double min_rot_vel;
    double acc_lim_x;
    double acc_lim_y;
    double acc_lim_theta;
    double acc_limit_trans;
//  double jerk_lim_trans;
//  double jerk_lim_rot;
    bool prune_plan;
    double xy_goal_tolerance;
    double yaw_goal_tolerance;
    double trans_stopped_vel;
    double rot_stopped_vel;
    bool restore_defaults;

    LocalPlannerLimits ()
    {
    }
    
    LocalPlannerLimits (double nmax_trans_vel, double nmin_trans_vel,
                        double nmax_vel_x, double nmin_vel_x, double nmax_vel_y,
                        double nmin_vel_y, double nmax_rot_vel,
                        double nmin_rot_vel,
                        double nacc_lim_x,
                        double nacc_lim_y,
                        double nacc_lim_theta,
                        double nacc_limit_trans,
                        double nxy_goal_tolerance,
                        double nyaw_goal_tolerance,
//      double njerk_lim_trans = -1,
//      double njerk_lim_rot = -1,
                        bool nprune_plan = true,
                        double ntrans_stopped_vel = 0.1,
                        double nrot_stopped_vel = 0.1)
        : max_trans_vel (nmax_trans_vel), min_trans_vel (nmin_trans_vel),
            max_vel_x (nmax_vel_x), min_vel_x (nmin_vel_x),
            max_vel_y (nmax_vel_y), min_vel_y (nmin_vel_y),
            max_rot_vel (nmax_rot_vel), min_rot_vel (nmin_rot_vel),
            acc_lim_x (nacc_lim_x), acc_lim_y (nacc_lim_y),
            acc_lim_theta (nacc_lim_theta), acc_limit_trans (nacc_limit_trans),
//        jerk_lim_trans(njerk_lim_trans),
//        jerk_lim_rot(njerk_lim_rot),
            prune_plan (nprune_plan), xy_goal_tolerance (nxy_goal_tolerance),
            yaw_goal_tolerance (nyaw_goal_tolerance),
            trans_stopped_vel (ntrans_stopped_vel),
            rot_stopped_vel (nrot_stopped_vel)
    {
    }
    
    ~LocalPlannerLimits ()
    {
    }
    
    /**
     * @brief  Get the acceleration limits of the robot
     * @return  The acceleration limits of the robot
     */
    Eigen::Vector3f
    getAccLimits ()
    {
      Eigen::Vector3f acc_limits;
      acc_limits[0] = acc_lim_x;
      acc_limits[1] = acc_lim_y;
      acc_limits[2] = acc_lim_theta;
      return acc_limits;
    }
    
  };

}
#endif // __LOCALPLANNERLIMITS_H__
