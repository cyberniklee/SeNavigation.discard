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
#include <stdio.h>
#include <string.h>
#include <Time/Utils.h>

namespace NS_Controller
{
  
  SpiComm::SpiComm (std::string device_name)
  {
    // TODO Auto-generated constructor stub
    dev_name = device_name;
    is_open = false;
  }
  
  SpiComm::~SpiComm ()
  {
    // TODO Auto-generated destructor stub
  }
  
  bool
  SpiComm::open ()
  {
    spi_dev = ::open (dev_name.c_str (), O_RDWR);
    if (spi_dev < 0)
    {
      return false;
    }
    
    is_open = true;
    return true;
  }
  
  void
  SpiComm::close ()
  {
    if (is_open)
      ::close (spi_dev);
  }
  
  bool
  SpiComm::transfer (unsigned char* tx_buf, unsigned char* rx_buf,
                     size_t tx_len, size_t rx_len)
  {
    if (!is_open)
      return false;
    
    boost::mutex::scoped_lock dev_mutex (dev_lock);
    
    if (::write (spi_dev, tx_buf, tx_len) != tx_len)
    {
      return false;
    }
    
    if (::read (spi_dev, rx_buf, rx_len) != rx_len)
    {
      return false;
    }
    
    return true;
  }
  
  bool
  SpiComm::getRegister (unsigned short address, int length,
                        unsigned char* bytes)
  {
    CntlSpiRequestType request;
    CntlSpiResponseType response;
    
    request.sync = SPI_REQUEST_SYNC;
    request.sequence = sequence++;
    request.type = SPI_REQUEST_TYPE;
    request.rdwr = SPI_READ;
    request.address = address;
    request.length = length;
    
    unsigned char req_buf[MAX_SPI_OFFSET + 16] = { 0 };
    int req_buf_len = 0;
    if (!serializeRequest (request, req_buf, req_buf_len))
    {
      return false;
    }
    
    unsigned char rep_buf[MAX_SPI_OFFSET + 16] = { 0 };
    int rep_buf_len = length + sizeof(response) - MAX_SPI_OFFSET;
    if (!transfer (req_buf, rep_buf, req_buf_len, rep_buf_len))
    {
      return false;
    }
    
    if (!deserializeResponse (rep_buf, rep_buf_len, response))
    {
      return false;
    }
    
    if (response.result != SPI_RESULT_SUCCESS)
    {
      return false;
    }
    
    if (response.length != length)
    {
      return false;
    }
    
    memcpy (bytes, response.data, response.length);
    
    return true;
  }
  
  bool
  SpiComm::setRegister (unsigned short address, unsigned char* bytes,
                        int length)
  {
    CntlSpiRequestType request;
    CntlSpiResponseType response;
    
    request.sync = SPI_REQUEST_SYNC;
    request.sequence = sequence++;
    request.type = SPI_REQUEST_TYPE;
    request.rdwr = SPI_WRITE;
    request.address = address;
    memcpy (request.data, bytes, length);
    request.length = length;
    
    unsigned char req_buf[MAX_SPI_OFFSET + 16] = { 0 };
    int req_buf_len = 0;
    if (!serializeRequest (request, req_buf, req_buf_len))
    {
      return false;
    }
    
    unsigned char rep_buf[MAX_SPI_OFFSET + 16] = { 0 };
    int rep_buf_len = sizeof(response) - MAX_SPI_OFFSET;
    if (!transfer (req_buf, rep_buf, req_buf_len, rep_buf_len))
    {
      return false;
    }
    
    if (!deserializeResponse (rep_buf, rep_buf_len, response))
    {
      return false;
    }
    
    if (response.result != SPI_RESULT_SUCCESS)
    {
      return false;
    }
    
    NS_NaviCommon::delay (50);
    
    return true;
  }
  
  double
  SpiComm::getFloat64Value (unsigned short address)
  {
    double result = 0.0f;
    
    getRegister (address, sizeof(double), (unsigned char*) &result);
    
    return result;
  }
  
  void
  SpiComm::setFloat64Value (unsigned short address, double value)
  {
    setRegister (address, (unsigned char*) &value, sizeof(double));
  }
  
  int
  SpiComm::getInt32Value (unsigned short address)
  {
    int result = 0;
    
    getRegister (address, sizeof(int), (unsigned char*) &result);
    
    return result;
  }
  
  void
  SpiComm::setInt32Value (unsigned short address, int value)
  {
    setRegister (address, (unsigned char*) &value, sizeof(int));
  }
  
  float
  SpiComm::getFloat32Value (unsigned short address)
  {
    float result = 0;
    
    getRegister (address, sizeof(float), (unsigned char*) &result);
    
    return result;
  }
  
  void
  SpiComm::setFloat32Value (unsigned short address, float value)
  {
    setRegister (address, (unsigned char*) &value, sizeof(float));
  }

} /* namespace NS_Controller */
