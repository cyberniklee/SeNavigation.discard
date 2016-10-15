/*
 * SpiComm.h
 *
 *  Created on: 2016年10月15日
 *      Author: seeing
 */

#ifndef CONTROLLER_COMMUNICATION_SPICOMM_H_
#define CONTROLLER_COMMUNICATION_SPICOMM_H_

#include "CommBase.h"

namespace NS_Controller {

class SpiComm: public CommBase {
public:
	SpiComm();
	virtual ~SpiComm();
};

} /* namespace NS_Controller */

#endif /* CONTROLLER_COMMUNICATION_SPICOMM_H_ */
