/*
 * SpiComm.h
 *
 *  Created on: 2016年10月15日
 *      Author: seeing
 */

#ifndef _CONTROLLER_COMMUNICATION_SPICOMM_H_
#define _CONTROLLER_COMMUNICATION_SPICOMM_H_

#include <string>
#include "Protocol.h"
#include <boost/thread/thread.hpp>

namespace NS_Controller {

class SpiComm {
public:
  SpiComm(std::string device_name);
  virtual ~SpiComm();

private:
  std::string dev_name;

  bool is_open;

  int spi_dev;

  boost::mutex dev_lock;

  bool transfer(unsigned char* tx_buf, unsigned char* rx_buf, size_t tx_len, size_t rx_len);

  unsigned short sequence;

public:
  bool open();

  void close();

  bool getRegister(unsigned short address, int length, unsigned char* bytes);
  bool setRegister(unsigned short address, unsigned char* bytes, int length);

  double getFloat64Value(unsigned short address);
  void setFloat64Value(unsigned short address, double value);

  int getInt32Value(unsigned short address);
  void setInt32Value(unsigned short address, int value);
};

} /* namespace NS_Controller */

#endif /* CONTROLLER_COMMUNICATION_SPICOMM_H_ */
