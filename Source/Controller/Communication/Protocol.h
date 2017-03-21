/*
 * Protocol.h
 *
 *  Created on: 2016年10月15日
 *      Author: seeing
 */

#ifndef _CONTROLLER_PROTOCOL_H_
#define _CONTROLLER_PROTOCOL_H_

#include <time.h>

namespace NS_Controller {

#define MAX_SPI_OFFSET 128

#define SPI_REQUEST_SYNC 0xA5
#define SPI_RESPONSE_SYNC 0xB5

#define SPI_REQUEST_TYPE 0x01
#define SPI_RESPONSE_TYPE 0x00

#define SPI_WRITE 0x01
#define SPI_READ 0x00

#define SPI_RESULT_SUCCESS 0x01
#define SPI_RESULT_FAILURE 0x00

enum{
	BASE_REG_PID_KP = 0x00,
	BASE_REG_PID_KI = 0x04,
	BASE_REG_PID_KD = 0x08,
	BASE_REG_PID_KO = 0x0C,

	BASE_REG_LINEAR_SPD = 0x20,
	BASE_REG_ANGULAR_SPD = 0x28,

	BASE_REG_DIST_PER_PULSE = 0x30,
	BASE_REG_WHEEL_TRACK = 0x38,
	BASE_REG_CNTL_RATE = 0x40,

	BASE_REG_ODOM_X = 0x50,
	BASE_REG_ODOM_Y = 0x58,
	BASE_REG_ODOM_THETA = 0x60,
	BASE_REG_ODOM_LINEAR_SPD = 0x70,
	BASE_REG_ODOM_ANGULAR_SPD = 0x78,

	BASE_REG_IMU_X = 0x80,
	BASE_REG_IMU_Y = 0x88,
	BASE_REG_IMU_THETA = 0x90,
	BASE_REG_IMU_LINEAR_SPD = 0xA0,
	BASE_REG_IMU_ANGULAR_SPD = 0xA8,
};

typedef struct {
	unsigned char sync;
	unsigned short sequence;
	unsigned char type;
	unsigned char rdwr;
	unsigned short address;
	unsigned short length;
	unsigned char data[MAX_SPI_OFFSET];
	unsigned char checksum;
}__attribute__((packed)) CntlSpiRequestType;

typedef struct {
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

bool serializeRequest(CntlSpiRequestType& request, unsigned char *buffer, int& length);

bool deserializeResponse(unsigned char *buffer, const int& length, CntlSpiResponseType& response);

} /* namespace NS_Controller */

#endif /* CONTROLLER_PROTOCOL_PROTOCOL_H_ */
