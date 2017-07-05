#ifndef _DWA_LOCAL_PLANNER_DWA_PLANNER_H_
#define _DWA_LOCAL_PLANNER_DWA_PLANNER_H_

#include <vector>
#include <Eigen/Core>

//for obstacle data access
#include "../../../../CostMap/CostMap2D/CostMap2D.h"

#include "../../TrajectoryLocalPlanner/Algorithm/Trajectory.h"
#include "../../TrajectoryLocalPlanner/Algorithm/LocalPlannerLimits.h"
#include "LocalPlannerUtil.h"
#include "SimpleTrajectoryGenerator.h"
#include "OscillationCostFunction.h"
#include "MapGridCostFunction.h"
#include "ObstacleCostFunction.h"
#include "SimpleScoredSamplingPlanner.h"

#include <DataSet/DataType/Path.h>

namespace NS_Planner {
  /**
   * @class DWAPlanner
   * @brief A class implementing a local planner using the Dynamic Window Approach
   */
  class DWAPlanner {
    public:
      /**
       * @brief  Constructor for the planner
       * @param name The name of the planner 
       * @param costmap_ros A pointer to the costmap instance the planner should use
       * @param global_frame the frame id of the tf frame to use
       */
      DWAPlanner(NS_Planner::LocalPlannerUtil *planner_util, bool sum_scores, double cheat_factor,
                 double sim_time = 1.0, double sim_granularity = 0.025, double angular_sim_granularity = 0.025,
                 double pdist_scale = 0.6, double gdist_scale = 0.8, double occdist_scale = 0.2,
                 double stop_time_buffer = 0.2, double oscillation_reset_dist = 0.05, double oscillation_reset_angle = 0.2,
                 double forward_point_dist = 0.325,
                 double max_trans_vel = 0.55,
                 double scaling_speed = 0.25,
                 double max_scaling_factor = 0.2,
                 int vx_samples = 3, int vy_samples = 10, int vth_samples = 20,
                 bool dwa = true,
                 double sim_period = 0.1);

      /**
       * @brief  Destructor for the planner
       */
      ~DWAPlanner() {}


      /**
       * @brief  Check if a trajectory is legal for a position/velocity pair
       * @param pos The robot's position
       * @param vel The robot's velocity
       * @param vel_samples The desired velocity
       * @return True if the trajectory is valid, false otherwise
       */
      bool checkTrajectory(
          const Eigen::Vector3f pos,
          const Eigen::Vector3f vel,
          const Eigen::Vector3f vel_samples);

      /**
       * @brief Given the current position and velocity of the robot, find the best trajectory to exectue
       * @param global_pose The current position of the robot 
       * @param global_vel The current velocity of the robot 
       * @param drive_velocities The velocities to send to the robot base
       * @return The highest scoring trajectory. A cost >= 0 means the trajectory is legal to execute.
       */
      NS_Planner::Trajectory findBestPath(
          NS_Transform::Stamped<NS_Transform::Pose> global_pose,
          NS_Transform::Stamped<NS_Transform::Pose> global_vel,
          NS_Transform::Stamped<NS_Transform::Pose>& drive_velocities,
          std::vector<NS_DataType::Point> footprint_spec);

      /**
       * @brief  Take in a new global plan for the local planner to follow, and adjust local costmaps
       * @param  new_plan The new global plan
       */
      void updatePlanAndLocalCosts(NS_Transform::Stamped<NS_Transform::Pose> global_pose,
          const std::vector<NS_DataType::PoseStamped>& new_plan);

      /**
       * @brief Get the period at which the local planner is expected to run
       * @return The simulation period
       */
      double getSimPeriod() { return sim_period_; }

      /**
       * @brief Compute the components and total cost for a map grid cell
       * @param cx The x coordinate of the cell in the map grid
       * @param cy The y coordinate of the cell in the map grid
       * @param path_cost Will be set to the path distance component of the cost function
       * @param goal_cost Will be set to the goal distance component of the cost function
       * @param occ_cost Will be set to the costmap value of the cell
       * @param total_cost Will be set to the value of the overall cost function, taking into account the scaling parameters
       * @return True if the cell is traversible and therefore a legal location for the robot to move to
       */
      bool getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost);

      /**
       * sets new plan and resets state
       */
      bool setPlan(const std::vector<NS_DataType::PoseStamped>& orig_global_plan);

    private:

      NS_Planner::LocalPlannerUtil *planner_util_;

      double stop_time_buffer_; ///< @brief How long before hitting something we're going to enforce that the robot stop
      double pdist_scale_, gdist_scale_, occdist_scale_;
      Eigen::Vector3f vsamples_;

      double sim_period_;///< @brief The number of seconds to use to compute max/min vels for dwa
      NS_Planner::Trajectory result_traj_;

      double forward_point_distance_;

      std::vector<NS_DataType::PoseStamped> global_plan_;

      boost::mutex configuration_mutex_;

      double cheat_factor_;

      // see constructor body for explanations
      NS_Planner::SimpleTrajectoryGenerator generator_;
      NS_Planner::OscillationCostFunction oscillation_costs_;
      NS_Planner::ObstacleCostFunction obstacle_costs_;
      NS_Planner::MapGridCostFunction path_costs_;
      NS_Planner::MapGridCostFunction goal_costs_;
      NS_Planner::MapGridCostFunction goal_front_costs_;
      NS_Planner::MapGridCostFunction alignment_costs_;

      NS_Planner::SimpleScoredSamplingPlanner scored_sampling_planner_;
  };
};
#endif
