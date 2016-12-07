/*
 * BaseLocalPlanner.h
 *
 *  Created on: 2016年12月2日
 *      Author: seeing
 */

#ifndef _BASELOCALPLANNER_H_
#define _BASELOCALPLANNER_H_

#include "../../Base/LocalPlannerBase.h"

namespace NS_Planner {

class BaseLocalPlanner: public LocalPlannerBase {
public:
	BaseLocalPlanner();
	virtual ~BaseLocalPlanner();

	virtual void onInitialize();

	virtual bool computeVelocityCommands(NS_DataType::Twist& cmd_vel);

	virtual bool isGoalReached();

	virtual bool setPlan(const std::vector<NS_DataType::PoseStamped>& plan);
};

} /* namespace NS_Planner */

#endif /* NAVIGATION_PLANNER_IMPLEMENTS_BASELOCALPLANNER_BASELOCALPLANNER_H_ */
