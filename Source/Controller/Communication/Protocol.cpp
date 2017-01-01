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

  for(int i = 0; i < request.length; i++)
  {
    *(buffer + position + i) = request.data[i];
    position++;
  }

  for(int i = 0; i < position; i++)
  {
    request.checksum ^= *(buffer + i);
  }

  memcpy(buffer + position, &request.checksum, sizeof(request.checksum));
  position += sizeof(request.checksum);

  return true;
}

bool deserializeResponse(unsigned char *buffer, const int& length, CntlSpiResponseType& response)
{
  if(buffer == NULL)
	return false;
  return true;
}

} /* namespace NS_Controller */
