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
    BASE_REG_CFG_DONE = 0x04,
    
    BASE_REG_PID_KP_RIGHT = 0x10,
    BASE_REG_PID_KI_RIGHT = 0x18,
    BASE_REG_PID_KD_RIGHT = 0x20,
    BASE_REG_PID_KO_RIGHT = 0x28,
    BASE_REG_PID_MAX_RIGHT = 0x30,
    BASE_REG_PID_MIN_RIGHT = 0x38,
    
    BASE_REG_PID_KP_LEFT = 0x40,
    BASE_REG_PID_KI_LEFT = 0x48,
    BASE_REG_PID_KD_LEFT = 0x50,
    BASE_REG_PID_KO_LEFT = 0x58,
    BASE_REG_PID_MAX_LEFT = 0x60,
    BASE_REG_PID_MIN_LEFT = 0x68,
    
    BASE_REG_ACCEL_LIMIT = 0x70,
    BASE_REG_CNTL_DURATION = 0x78,
    
    BASE_REG_WHEEL_DIAMETER = 0x80,
    BASE_REG_WHEEL_TRACK = 0x88,
    BASE_REG_ENCODER_RESOLUTION = 0x90,
    BASE_REG_GEAR_REDUCTION = 0x98,
    
    BASE_REG_VEL_TIMEOUT = 0xA0,
    BASE_REG_LINEAR_V = 0xA8,
    BASE_REG_ANGULAR_V = 0xB0,
    BASE_REG_V_SETTED = 0xB8,
    
    BASE_REG_ODOM_X = 0xC0,
    BASE_REG_ODOM_Y = 0xC8,
    BASE_REG_ODOM_THETA = 0xD0,
    BASE_REG_ODOM_LINEAR_VEL = 0xD8,
    BASE_REG_ODOM_ANGULAR_VEL = 0xE0,
    
    BASE_REG_IMU_ROLL = 0xF0,
    BASE_REG_IMU_PITCH = 0xF8,
    BASE_REG_IMU_YAW = 0x100,

    BASE_REG_END = 0x110,
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
