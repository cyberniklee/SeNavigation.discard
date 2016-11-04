/*
 * CostmapApplication.h
 *
 *  Created on: 2016年11月3日
 *      Author: seeing
 */

#ifndef _COSTMAP_COSTMAPAPPLICATION_H_
#define _COSTMAP_COSTMAPAPPLICATION_H_

#include "../../Application/Application.h"

namespace NS_CostMap {

class CostmapApplication: public Application {
public:
	CostmapApplication();
	virtual ~CostmapApplication();
private:
	void loadParameters();
public:
	virtual void initialize();
	virtual void run();
	virtual void quit();
};

} /* namespace NS_CostMap */

#endif /* NAVIGATION_COSTMAP_COSTMAPAPPLICATION_H_ */
