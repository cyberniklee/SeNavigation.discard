/*
 * NavigationApplication.cpp
 *
 *  Created on: 2016年12月3日
 *      Author: seeing
 */

#include "NavigationApplication.h"
#include "Planner/Implements/BaseLocalPlanner/BaseLocalPlanner.h"
#include "Planner/Implements/DwaLocalPlanner/DwaLocalPlanner.h"
#include "Planner/Implements/GlobalPlanner/GlobalPlanner.h"


namespace NS_Navigation {

NavigationApplication::NavigationApplication() {
	// TODO Auto-generated constructor stub

}

NavigationApplication::~NavigationApplication()
{
	// TODO Auto-generated destructor stub
}

void NavigationApplication::loadParameters()
{

}

void NavigationApplication::planLoop()
{
  while(running)
  {

  }
}

void NavigationApplication::initialize()
{
  loadParameters();

  //set up plan triple buffer
  global_planner_plan = new std::vector<NS_DataType::PoseStamped>();
  latest_plan = new std::vector<NS_DataType::PoseStamped>();
  local_planner_plan = new std::vector<NS_DataType::PoseStamped>();

  global_costmap = new NS_CostMap::CostmapWrapper(dispitcher, service);
  local_costmap = new NS_CostMap::CostmapWrapper(dispitcher, service);

  //load global planner
  if(global_planner_type_ == "base_global_planner")
  {
    global_planner = new NS_Planner::GlobalPlanner();
  }else{
    global_planner = new NS_Planner::GlobalPlanner();
  }

  //load local planner
  if(local_planner_type_ == "base_local_planner")
  {
    local_planner = new NS_Planner::BaseLocalPlanner();
  }else if(local_planner_type_ == "dwa_local_planner")
  {
    local_planner = new NS_Planner::DwaLocalPlanner();
  }else{
    local_planner = new NS_Planner::BaseLocalPlanner();
  }

  global_planner->initialize(global_costmap, dispitcher, service);
  local_planner->initialize(local_costmap, dispitcher, service);

  initialized = true;
}

void NavigationApplication::run()
{
  running = true;
  plan_thread = boost::thread(boost::bind(&NavigationApplication::planLoop, this));
}

void NavigationApplication::quit()
{
  running = false;
  plan_thread.join();
}

} /* namespace NS_Navigation */
