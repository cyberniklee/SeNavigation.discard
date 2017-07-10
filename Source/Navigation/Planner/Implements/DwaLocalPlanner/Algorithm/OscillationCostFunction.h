#ifndef _DWA_LOCAL_PLANNER_OSCILLATION_COST_FUNCTION_H_
#define _DWA_LOCAL_PLANNER_OSCILLATION_COST_FUNCTION_H_

#include "TrajectoryCostFunction.h"
#include <Eigen/Core>

namespace NS_Planner
{
  
  class OscillationCostFunction: public TrajectoryCostFunction
  {
  public:
    OscillationCostFunction ();
    virtual
    ~OscillationCostFunction ();

    double
    scoreTrajectory (Trajectory &traj);

    bool
    prepare ()
    {
      return true;
    }
    ;

    /**
     * @brief  Reset the oscillation flags for the local planner
     */
    void
    resetOscillationFlags ();

    void
    updateOscillationFlags (Eigen::Vector3f pos, NS_Planner::Trajectory* traj,
                            double min_vel_trans);

    void
    setOscillationResetDist (double dist, double angle);

  private:
    
    void
    resetOscillationFlagsIfPossible (const Eigen::Vector3f& pos,
                                     const Eigen::Vector3f& prev);

    /**
     * @brief  Given a trajectory that's selected, set flags if needed to
     * prevent the robot from oscillating
     * @param  t The selected trajectory
     * @return True if a flag was set, false otherwise
     */
    bool
    setOscillationFlags (NS_Planner::Trajectory* t, double min_vel_trans);

    // flags
    bool strafe_pos_only_, strafe_neg_only_, strafing_pos_, strafing_neg_;
    bool rot_pos_only_, rot_neg_only_, rotating_pos_, rotating_neg_;
    bool forward_pos_only_, forward_neg_only_, forward_pos_, forward_neg_;

    // param
    double oscillation_reset_dist_, oscillation_reset_angle_;

    Eigen::Vector3f prev_stationary_pos_;
  };

} /* namespace base_local_planner */
#endif /* OSCILLATION_COST_FUNCTION_H_ */
