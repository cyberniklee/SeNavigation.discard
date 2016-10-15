/*
 * CommBase.h
 *
 *  Created on: 2016年10月15日
 *      Author: seeing
 */

#ifndef _CONTROLLER_COMMUNICATION_COMMBASE_H_
#define _CONTROLLER_COMMUNICATION_COMMBASE_H_

namespace NS_Controller {

class CommBase {
public:
	CommBase();
	virtual ~CommBase();

public:
	bool initialize();
};

} /* namespace NS_Controller */

#endif /* CONTROLLER_COMMUNICATION_COMMBASE_H_ */
