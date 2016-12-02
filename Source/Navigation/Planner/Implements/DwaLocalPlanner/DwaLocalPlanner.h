/*
 * DwaLocalPlanner.h
 *
 *  Created on: 2016年12月2日
 *      Author: seeing
 */

#ifndef _DWALOCALPLANNER_H_
#define _DWALOCALPLANNER_H_

#include "../../Base/LocalPlannerBase.h"

namespace NS_Planner {

class DwaLocalPlanner: public LocalPlannerBase {
public:
	DwaLocalPlanner();
	virtual ~DwaLocalPlanner();
};

} /* namespace NS_Planner */

#endif /* NAVIGATION_PLANNER_IMPLEMENTS_DWALOCALPLANNER_DWALOCALPLANNER_H_ */
