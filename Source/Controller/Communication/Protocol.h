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



typedef struct {
	unsigned char sync;
	unsigned short sequence;
	unsigned char type;
	unsigned char rdwr;
	unsigned short address;
	unsigned short length;
	unsigned char data[MAX_SPI_OFFSET];
	unsigned char checksum;
}CntlSpiRequestType;

typedef struct {
	unsigned char sync;
	unsigned short sequence;
	unsigned char type;
	unsigned char result;
	time_t timestamp;
	unsigned short length;
	unsigned char data[MAX_SPI_OFFSET];
	unsigned char checksum;
}CntlSpiResponseType;

bool serializeRequest(CntlSpiRequestType& request, unsigned char *buffer, int& length);

bool deserializeResponse(unsigned char *buffer, const int& length, CntlSpiResponseType& response);

} /* namespace NS_Controller */

#endif /* CONTROLLER_PROTOCOL_PROTOCOL_H_ */
