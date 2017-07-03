/*
 * LocalPlannerBase.h
 *
 *  Created on: 2016年12月2日
 *      Author: seeing
 */

#ifndef _LOCALPLANNERBASE_H_
#define _LOCALPLANNERBASE_H_

#include <DataSet/Dispitcher.h>
#include <Service/Service.h>
#include <DataSet/DataType/PoseStamped.h>
#include <DataSet/DataType/Twist.h>

namespace NS_Planner {

class LocalPlannerBase {
public:
	LocalPlannerBase(){};
	virtual ~LocalPlannerBase(){};

public:
	void initialize(NS_CostMap::CostmapWrapper* costmap_, NS_NaviCommon::Dispitcher* dispitcher_, NS_NaviCommon::Service* service_)
	{
	  costmap = costmap_;
          service = service_;
          dispitcher = dispitcher_;
          onInitialize();
	};

	virtual void onInitialize() = 0;

	virtual bool computeVelocityCommands(NS_DataType::Twist& cmd_vel) = 0;

	virtual bool isGoalReached() = 0;

	virtual bool setPlan(const std::vector<NS_DataType::PoseStamped>& plan) = 0;

private:
	NS_NaviCommon::Dispitcher* dispitcher;
	NS_NaviCommon::Service* service;
	NS_CostMap::CostmapWrapper* costmap;
};

} /* namespace NS_Planner */

#endif /* NAVIGATION_PLANNER_BASE_LOCALPLANNERBASE_H_ */
