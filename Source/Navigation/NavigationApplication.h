/*
 * NavigationApplication.h
 *
 *  Created on: 2016年12月3日
 *      Author: seeing
 */

#ifndef _NAVIGATIONAPPLICATION_H_
#define _NAVIGATIONAPPLICATION_H_

#include "../Application/Application.h"
#include "CostMap/CostmapWrapper.h"
#include <DataSet/DataType/PoseStamped.h>
#include "Planner/Base/GlobalPlannerBase.h"
#include "Planner/Base/LocalPlannerBase.h"
#include <boost/thread/thread.hpp>
#include <DataSet/DataType/PoseStamped.h>
#include <vector>

namespace NS_Navigation
{
  
  enum NaviState
  {
    PLANNING, CONTROLLING, CLEARING
  };
  
  class NavigationApplication: public Application
  {
  public:
    NavigationApplication ();
    virtual
    ~NavigationApplication ();
  private:
    void
    loadParameters ();

    void
    planLoop ();

    bool
    makePlan (const NS_DataType::PoseStamped& goal,
              std::vector<NS_DataType::PoseStamped>& plan);

    void
    goalCallback (NS_DataType::DataBase* target_goal);

    NS_DataType::PoseStamped
    goalToGlobalFrame (NS_DataType::PoseStamped& goal);

    void
    publishZeroVelocity ();
    bool
    isQuaternionValid (const NS_DataType::Quaternion& q);
    double
    distance (const NS_DataType::PoseStamped& p1,
              const NS_DataType::PoseStamped& p2);
  private:
    std::string global_planner_type_;
    std::string local_planner_type_;

  private:
    //set up plan triple buffer
    std::vector<NS_DataType::PoseStamped>* global_planner_plan;
    std::vector<NS_DataType::PoseStamped>* latest_plan;
    std::vector<NS_DataType::PoseStamped>* local_planner_plan;

    NS_CostMap::CostmapWrapper* global_costmap;
    NS_CostMap::CostmapWrapper* local_costmap;

    NS_Planner::GlobalPlannerBase* global_planner;
    NS_Planner::LocalPlannerBase* local_planner;

    NS_DataType::PoseStamped goal;

    bool new_goal_trigger;

    boost::thread plan_thread;

    NaviState state;
  public:
    virtual void
    initialize ();
    virtual void
    run ();
    virtual void
    quit ();
  };

} /* namespace NS_Navigation */

#endif /* NAVIGATION_NAVIGATIONAPPLICATION_H_ */
