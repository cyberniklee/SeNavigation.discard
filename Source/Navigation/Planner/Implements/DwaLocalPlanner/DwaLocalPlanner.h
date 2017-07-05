#ifndef _DWA_LOCAL_PLANNER_
#define _DWA_LOCAL_PLANNER_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <Geometry/Angles.h>

#include <DataSet/DataType/Odometry.h>

#include "../../../CostMap/CostMap2D/CostMap2D.h"
#include "../../Base/LocalPlannerBase.h"


#include "Algorithm/LatchedStopRotateController.h"

#include "../TrajectoryLocalPlanner/Algorithm/OdometryHelper.h"

#include "Algorithm/DwaPlanner.h"

namespace NS_Planner {
  /**
   * @class DWAPlannerROS
   * @brief ROS Wrapper for the DWAPlanner that adheres to the
   * BaseLocalPlanner interface and can be used as a plugin for move_base.
   */
  class DWAPlannerROS : public LocalPlannerBase {
    public:
      /**
       * @brief  Constructor for DWAPlannerROS wrapper
       */
      DWAPlannerROS();

      /**
       * @brief  Constructs the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      virtual void onInitialize();

      /**
       * @brief  Destructor for the wrapper
       */
      ~DWAPlannerROS();

      /**
       * @brief  Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool computeVelocityCommands(NS_DataType::Twist& cmd_vel);


      /**
       * @brief  Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base, using dynamic window approach
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool dwaComputeVelocityCommands(NS_Transform::Stamped<NS_Transform::Pose>& global_pose, NS_DataType::Twist& cmd_vel);

      /**
       * @brief  Set the plan that the controller is following
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<NS_DataType::PoseStamped>& orig_global_plan);

      /**
       * @brief  Check if the goal pose has been achieved
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();



      bool isInitialized() {
        return initialized_;
      }

    private:

      NS_Planner::LocalPlannerUtil planner_util_;

      boost::shared_ptr<DWAPlanner> dp_; ///< @brief The trajectory controller

      bool setup_;
      NS_Transform::Stamped<NS_Transform::Pose> current_pose_;

      NS_Planner::LatchedStopRotateController latchedStopRotateController_;


      bool initialized_;


      NS_Planner::OdometryHelper odom_helper_;
  };
};
#endif
