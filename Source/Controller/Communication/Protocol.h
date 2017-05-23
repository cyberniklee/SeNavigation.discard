/*
 * Protocol.h
 *
 *  Created on: 2016年10月15日
 *      Author: seeing
 */

#ifndef _CONTROLLER_PROTOCOL_H_
#define _CONTROLLER_PROTOCOL_H_

#include <time.h>

namespace NS_Controller
{
  
#define MAX_SPI_OFFSET 128
  
#define SPI_REQUEST_SYNC 0xA5
#define SPI_RESPONSE_SYNC 0xB5
  
#define SPI_REQUEST_TYPE 0x01
#define SPI_RESPONSE_TYPE 0x00
  
#define SPI_WRITE 0x01
#define SPI_READ 0x00
  
#define SPI_RESULT_SUCCESS 0x01
#define SPI_RESULT_FAILURE 0x00
  
  enum
  {
    BASE_REG_TEST = 0x00,

    BASE_REG_PID_KP = 0x10,
    BASE_REG_PID_KI = 0x14,
    BASE_REG_PID_KD = 0x18,
    BASE_REG_PID_KO = 0x1C,

    BASE_REG_TOLERANCE = 0x20,

    BASE_REG_TICKS_PER_METER = 0x40,
    BASE_REG_WHEEL_TRACK = 0x48,

    BASE_REG_TARGET_X = 0x50,
    BASE_REG_TARGET_Y = 0x58,
    BASE_REG_TARGET_THETA = 0x60,
    BASE_REG_TARGET_SETTED = 0x68,
    BASE_REG_TARGET_REACHED = 0x70,

    BASE_REG_ODOM_X = 0x80,
    BASE_REG_ODOM_Y = 0x88,
    BASE_REG_ODOM_THETA = 0x90,

    BASE_REG_END = 0xA0,
  };
  
  typedef struct
  {
    unsigned char sync;
    unsigned short sequence;
    unsigned char type;
    unsigned char rdwr;
    unsigned short address;
    unsigned short length;
    unsigned char data[MAX_SPI_OFFSET];
    unsigned char checksum;
  }__attribute__((packed)) CntlSpiRequestType;
  
  typedef struct
  {
    unsigned char sync;
    unsigned short sequence;
    unsigned char type;
    unsigned char result;
    unsigned long timestamp;
    unsigned short timestamp_ms;
    unsigned short length;
    unsigned char data[MAX_SPI_OFFSET];
    unsigned char checksum;
  }__attribute__((packed)) CntlSpiResponseType;
  
  bool
  serializeRequest (CntlSpiRequestType& request, unsigned char *buffer,
                    int& length);
  
  bool
  deserializeResponse (unsigned char *buffer, const int& length,
                       CntlSpiResponseType& response);

} /* namespace NS_Controller */

#endif /* CONTROLLER_PROTOCOL_PROTOCOL_H_ */
