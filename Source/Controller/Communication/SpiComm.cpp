/*
 * SpiComm.cpp
 *
 *  Created on: 2016年10月15日
 *      Author: seeing
 */

#include "SpiComm.h"
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>

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
  spi_dev = ::open(dev_name.c_str(), O_RDWR);
  if(spi_dev < 0)
  {
    return false;
  }

  return true;
}

void SpiComm::close()
{
  ::close(spi_dev);
}

bool SpiComm::transfer(unsigned char* tx_buf, unsigned char* rx_buf, size_t tx_len, size_t rx_len)
{

}

} /* namespace NS_Controller */
