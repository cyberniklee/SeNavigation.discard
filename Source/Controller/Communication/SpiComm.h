/*
 * SpiComm.h
 *
 *  Created on: 2016年10月15日
 *      Author: seeing
 */

#ifndef CONTROLLER_COMMUNICATION_SPICOMM_H_
#define CONTROLLER_COMMUNICATION_SPICOMM_H_

#include <string>

namespace NS_Controller {

class SpiComm {
public:
	SpiComm(std::string device_name);
	virtual ~SpiComm();

private:
	std::string dev_name;

	int spi_dev;

public:
	bool open();

	void close();

	bool transfer(unsigned char* tx_buf, unsigned char* rx_buf, size_t tx_len, size_t rx_len);
};

} /* namespace NS_Controller */

#endif /* CONTROLLER_COMMUNICATION_SPICOMM_H_ */
