/*
 * BaseLocalPlanner.h
 *
 *  Created on: 2016年12月2日
 *      Author: seeing
 */

#ifndef _BASELOCALPLANNER_H_
#define _BASELOCALPLANNER_H_

namespace NS_Planner {

class BaseLocalPlanner: public LocalPlannerBase {
public:
	BaseLocalPlanner();
	virtual ~BaseLocalPlanner();
};

} /* namespace NS_Planner */

#endif /* NAVIGATION_PLANNER_IMPLEMENTS_BASELOCALPLANNER_BASELOCALPLANNER_H_ */
