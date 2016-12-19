/*
 * SpiComm.cpp
 *
 *  Created on: 2016年10月15日
 *      Author: seeing
 */

#include "SpiComm.h"

namespace NS_Controller {

SpiComm::SpiComm(std::string device_name) {
  // TODO Auto-generated constructor stub
  dev_name = device_name;
}

SpiComm::~SpiComm() {
	// TODO Auto-generated destructor stub
}

bool SpiComm::open()
{

}

void SpiComm::close()
{

}

bool SpiComm::transfer(unsigned char* tx_buf, unsigned char* rx_buf, size_t tx_len, size_t rx_len)
{

}

} /* namespace NS_Controller */
