#ifndef _DWA_LOCAL_PLANNER_UTIL_H_
#define _DWA_LOCAL_PLANNER_UTIL_H_

#include <boost/thread.hpp>

#include "../../../../CostMap/CostMap2D/CostMap2D.h"
#include <Transform/DataTypes.h>
#include <Service/Service.h>
#include "../../TrajectoryLocalPlanner/Algorithm/LocalPlannerLimits.h"

namespace NS_Planner
{
  
  /**
   * @class LocalPlannerUtil
   * @brief Helper class implementing infrastructure code many local planner implementations may need.
   */
  class LocalPlannerUtil
  {
    
  private:
    // things we get from move_base
    std::string name_;
    std::string global_frame_;

    NS_CostMap::Costmap2D* costmap_;
    NS_NaviCommon::Service* service_;

    std::vector<NS_DataType::PoseStamped> global_plan_;

    boost::mutex limits_configuration_mutex_;
    bool setup_;
    LocalPlannerLimits default_limits_;
    LocalPlannerLimits limits_;
    bool initialized_;

  public:
    
    /**
     * @brief  Callback to update the local planner's parameters
     */
    //todo: need re-coding this part
    void
    reconfigureCB (LocalPlannerLimits &config, bool restore_defaults);

    LocalPlannerUtil ()
        : initialized_ (false)
    {
    }
    
    ~LocalPlannerUtil ()
    {
    }
    
    void
    initialize (NS_NaviCommon::Service* service,
                NS_CostMap::Costmap2D* costmap);

    bool
    getGoal (NS_Transform::Stamped<NS_Transform::Pose>& goal_pose);

    bool
    setPlan (const std::vector<NS_DataType::PoseStamped>& orig_global_plan);

    bool
    getLocalPlan (NS_Transform::Stamped<NS_Transform::Pose>& global_pose,
                  std::vector<NS_DataType::PoseStamped>& transformed_plan);

    NS_CostMap::Costmap2D*
    getCostmap ();

    LocalPlannerLimits
    getCurrentLimits ();

    std::string
    getGlobalFrame ()
    {
      return global_frame_;
    }
  };

}
;

#endif /* ABSTRACT_LOCAL_PLANNER_ODOM_H_ */
