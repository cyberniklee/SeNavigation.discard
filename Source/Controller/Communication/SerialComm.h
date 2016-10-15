/*
 * SerialComm.h
 *
 *  Created on: 2016年10月15日
 *      Author: seeing
 */

#ifndef _CONTROLLER_COMMUNICATION_SERIALCOMM_H_
#define _CONTROLLER_COMMUNICATION_SERIALCOMM_H_

#include "CommBase.h"

namespace NS_Controller {

class SerialComm: public CommBase {
public:
	SerialComm();
	virtual ~SerialComm();
};

} /* namespace NS_Controller */

#endif /* CONTROLLER_COMMUNICATION_SERIALCOMM_H_ */
