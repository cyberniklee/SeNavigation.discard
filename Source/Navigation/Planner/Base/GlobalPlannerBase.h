/*
 * GlobalPlannerBase.h
 *
 *  Created on: 2016年12月2日
 *      Author: seeing
 */

#ifndef _GLOBALPLANNERBASE_H_
#define _GLOBALPLANNERBASE_H_

#include <DataSet/DataType/PoseStamped.h>
#include <DataSet/Dispitcher.h>
#include <Service/Service.h>
#include "../../CostMap/CostmapWrapper.h"

namespace NS_Planner {

class GlobalPlannerBase {
public:
	GlobalPlannerBase(){};
	virtual ~GlobalPlannerBase(){};

public:
	void initialize(NS_CostMap::CostmapWrapper* costmap_, NS_NaviCommon::Dispitcher* dispitcher_, NS_NaviCommon::Service* service_)
	{
		dispitcher = dispitcher_;
		service = service_;
		costmap = costmap_;
		onInitialize();
	};

	virtual void onInitialize() = 0;

	virtual bool makePlan(const NS_DataType::PoseStamped& start,
			const NS_DataType::PoseStamped& goal, std::vector<NS_DataType::PoseStamped>& plan) = 0;

	virtual bool makePlan(const NS_DataType::PoseStamped& start,
			const NS_DataType::PoseStamped& goal, std::vector<NS_DataType::PoseStamped>& plan,
			double& cost)
	{
		cost = 0;
		makePlan(start, goal, plan);
	};
protected:
	NS_NaviCommon::Dispitcher* dispitcher;
	NS_NaviCommon::Service* service;
	NS_CostMap::CostmapWrapper* costmap;
};

} /* namespace NS_Planner */

#endif /* NAVIGATION_PLANNER_BASE_GLOBALPLANNERBASE_H_ */
