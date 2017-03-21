/*
 * Protocol.cpp
 *
 *  Created on: 2016年10月15日
 *      Author: seeing
 */

#include "Protocol.h"
#include <stdio.h>
#include <string.h>

namespace NS_Controller {

bool serializeRequest(CntlSpiRequestType& request, unsigned char *buffer, int& length)
{
  if(buffer == NULL)
    return false;

  int position = 0;

  memcpy(buffer + position, &request.sync, sizeof(request.sync));
  position += sizeof(request.sync);

  memcpy(buffer + position, &request.sequence, sizeof(request.sequence));
  position += sizeof(request.sequence);

  memcpy(buffer + position, &request.type, sizeof(request.type));
  position += sizeof(request.type);

  memcpy(buffer + position, &request.rdwr, sizeof(request.rdwr));
  position += sizeof(request.rdwr);

  memcpy(buffer + position, &request.address, sizeof(request.address));
  position += sizeof(request.address);

  memcpy(buffer + position, &request.length, sizeof(request.length));
  position += sizeof(request.length);

  if(request.rdwr == SPI_WRITE)
  {
    memcpy(buffer + position, request.data, request.length);
    position += request.length;
  }

  request.checksum = 0;
  for(int i = 0; i < position; i++)
  {
    request.checksum ^= *(buffer + i);
  }

  memcpy(buffer + position, &request.checksum, sizeof(request.checksum));
  position += sizeof(request.checksum);

  length = position;

  return true;
}

bool deserializeResponse(unsigned char *buffer, const int& length, CntlSpiResponseType& response)
{
  if(buffer == NULL)
	return false;

  int position = 0;

  memcpy(&response.sync, buffer + position, sizeof(response.sync));
  position += sizeof(response.sync);

  memcpy(&response.sequence, buffer + position, sizeof(response.sequence));
  position += sizeof(response.sequence);

  memcpy(&response.type, buffer + position, sizeof(response.type));
  position += sizeof(response.type);

  memcpy(&response.result, buffer + position, sizeof(response.result));
  position += sizeof(response.result);

  memcpy(&response.timestamp, buffer + position, sizeof(response.timestamp));
  position += sizeof(response.timestamp);

  memcpy(&response.timestamp_ms, buffer + position, sizeof(response.timestamp_ms));
  position += sizeof(response.timestamp_ms);

  memcpy(&response.length, buffer + position, sizeof(response.length));
  position += sizeof(response.length);

  if(response.length > MAX_SPI_OFFSET)
  {
    return false;
  }

  memcpy(&response.data, buffer + position, response.length);
  position += response.length;

  unsigned char checksum = 0;
  for(int i = 0; i < position; i++)
  {
    checksum ^= *(buffer + i);
  }

  memcpy(&response.checksum, buffer + position, sizeof(response.checksum));
  position += sizeof(response.checksum);

  if(checksum != response.checksum)
  {
    return false;
  }

  memcpy(&response.checksum, buffer + position, sizeof(response.checksum));
  position += sizeof(response.checksum);

  return true;
}

} /* namespace NS_Controller */
