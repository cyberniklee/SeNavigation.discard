/*
 * GlobalPlanner.h
 *
 *  Created on: 2016年12月2日
 *      Author: seeing
 */

#ifndef _GLOBALPLANNER_H_
#define _GLOBALPLANNER_H_

#include "../../Base/GlobalPlannerBase.h"

namespace NS_Planner {

class GlobalPlanner: public GlobalPlannerBase {
public:
	GlobalPlanner();
	virtual ~GlobalPlanner();
};

} /* namespace NS_Planner */

#endif /* NAVIGATION_PLANNER_IMPLEMENTS_GLOBALPLANNER_GLOBALPLANNER_H_ */
