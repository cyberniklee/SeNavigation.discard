
#include <stdio.h>
#include <iostream>
#include "SelidarDriver.h"
#include "SelidarTypes.h"
#include <Time/Utils.h>

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

#ifndef _countof
#define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))
#endif

using std::cin;
using std::cout;
using std::endl;


namespace NS_Selidar {


// Serial Driver Impl

SelidarDriver::SelidarDriver()
    : connected(false),
	  scanning(false)
{
  rxtx = new Serial();
}

SelidarDriver::~SelidarDriver()
{
  // force disconnection
  disconnect();

  delete rxtx;
}

int SelidarDriver::connect(const char * port_path, unsigned int baudrate, unsigned int flag)
{
  if (isConnected()) return Denied;

  if (!rxtx) return Invalid;

  {
    boost::mutex::scoped_lock auto_lock(rxtx_lock);

    // establish the serial connection...
    if (!rxtx->bind(port_path, baudrate)  ||  !rxtx->open()) {
      return Invalid;
    }

    rxtx->flush(0);
  }

  connected = true;

  return Success;
}

void SelidarDriver::disconnect()
{
  if (!connected) return ;
  stop();
  rxtx->close();
}

bool SelidarDriver::isConnected()
{
  return connected;
}

int SelidarDriver::sendCommand(unsigned char cmd)
{
  unsigned char pkt_header[16] = {0};
  unsigned char pkg[128] = {0};
  int pkg_size = 0;
  SelidarPacketHead * header = reinterpret_cast<SelidarPacketHead * >(pkt_header);
  unsigned char checksum = 0;

  if(!connected)
    return Failure;

  header->sync_word = SELIDAR_CMD_SYNC_BYTE;
  header->cmd_word = cmd;
  header->payload_len = 0;
  header->length = sizeof(SelidarPacketHead) + 1;

  for(size_t i = 0; i < sizeof(SelidarPacketHead); i++)
  {
    checksum ^= pkt_header[i];
  }

  memcpy(pkg, pkt_header, sizeof(SelidarPacketHead));
  pkg_size += sizeof(SelidarPacketHead);

  memcpy(pkg + pkg_size, &checksum, 1);
  pkg_size++;

  rxtx->senddata(pkg, pkg_size);

  return Success;
}


int SelidarDriver::reset(unsigned int timeout)
{
  int ans;

  {
    boost::mutex::scoped_lock auto_lock(rxtx_lock);

    if (IS_FAIL(ans = sendCommand(ResetReq))) {
      return ans;
    }
  }

  return Success;
}

int SelidarDriver::stop(unsigned int timeout)
{
  int ans;
  disableDataGrabbing();

  {
    boost::mutex::scoped_lock auto_lock(rxtx_lock);

    if (IS_FAIL(ans = sendCommand(StopReq))) {
      return ans;
    }
  }

  return Success;
}

void SelidarDriver::disableDataGrabbing()
{
  scanning = false;
  cache_thread.join();
}

int SelidarDriver::waitResponseHeader(SelidarPacketHead* header, unsigned int timeout)
{
  int recvPos = 0;
  unsigned int startTs = NS_NaviCommon::getMs();
  unsigned char  recvBuffer[sizeof(SelidarPacketHead)];
  unsigned char  *headerBuffer = reinterpret_cast<unsigned char *>(header);
  unsigned int waitTime;

  while ((waitTime = NS_NaviCommon::getMs() - startTs) <= timeout) {
    size_t remainSize = sizeof(SelidarPacketHead) - recvPos;
    size_t recvSize;
    int ans = rxtx->waitfordata(remainSize, timeout - waitTime, &recvSize);
    if (ans == Serial::ANS_DEV_ERR) {
      return Failure;
    }
    else if (ans == Serial::ANS_TIMEOUT) {
      return Timeout;
    }

    if(recvSize > remainSize) recvSize = remainSize;

    rxtx->recvdata(recvBuffer, recvSize);

    for (size_t pos = 0; pos < recvSize; ++pos) {
      unsigned char currentByte = recvBuffer[pos];

      if (recvPos == 0)
      {
        if (currentByte != SELIDAR_CMD_SYNC_BYTE)
        continue;
      }
      headerBuffer[recvPos++] = currentByte;
      if (recvPos == sizeof(SelidarPacketHead)) {
        return Success;
      }
    }
  }
  return Timeout;
}

int SelidarDriver::getHealth(SelidarHealth & health_info, unsigned int timeout)
{
  int  ans;

  if (!isConnected()) return Failure;

  disableDataGrabbing();

  {
    boost::mutex::scoped_lock auto_lock(rxtx_lock);

    if (IS_FAIL(ans = sendCommand(GetHealthReq))) {
      return ans;
    }


    SelidarPacketHead response_header;
    if (IS_FAIL(ans = waitResponseHeader(&response_header, timeout))) {
      return ans;
    }

    // verify whether we got a correct header
    if (response_header.cmd.bits.dir != Lidar2Host) {
      return Invalid;
    }

    if (response_header.cmd.bits.cmd_word != GetHealthRep) {
      return Invalid;
    }

    size_t data_size = sizeof(SelidarHealth) - sizeof(SelidarPacketHead) + 1;

    if (rxtx->waitfordata(data_size, timeout) != Serial::ANS_OK) {
      return Timeout;
    }

    unsigned char health_data[128] = {0};

    rxtx->recvdata(health_data, data_size);

    health_info.head = response_header;
    memcpy(&health_info + sizeof(SelidarPacketHead), health_data, data_size - 1);

    unsigned char checksum = 0;

    for (size_t i = 0; i < sizeof(SelidarHealth); i++)
    {
      checksum ^= ((unsigned char*)&health_info + i);
    }

    if(checksum != health_data[data_size - 1])
    {
      return BadCRC;
    }

  }

  return Success;
}

int SelidarDriver::getDeviceInfo(SelidarInfo & info, unsigned int timeout)
{
  int  ans;

  if (!isConnected()) return Failure;

  disableDataGrabbing();

  {
	boost::mutex::scoped_lock auto_lock(rxtx_lock);

	if (IS_FAIL(ans = sendCommand(GetInfoReq))) {
	  return ans;
	}


	SelidarPacketHead response_header;
	if (IS_FAIL(ans = waitResponseHeader(&response_header, timeout))) {
	  return ans;
	}

	// verify whether we got a correct header
	if (response_header.cmd.bits.dir != Lidar2Host) {
	  return Invalid;
	}

	if (response_header.cmd.bits.cmd_word != GetInfoRep) {
	  return Invalid;
	}

	size_t data_size = sizeof(SelidarInfo) - sizeof(SelidarPacketHead) + 1;

	if (rxtx->waitfordata(data_size, timeout) != Serial::ANS_OK) {
	  return Timeout;
	}

	unsigned char info_data[128] = {0};

	rxtx->recvdata(info_data, data_size);

	info.head = response_header;
	memcpy(&info + sizeof(SelidarPacketHead), info_data, data_size - 1);

	unsigned char checksum = 0;

	for (size_t i = 0; i < sizeof(SelidarInfo); i++)
	{
	  checksum ^= ((unsigned char*)&info + i);
	}

	if(checksum != info_data[data_size - 1])
	{
	  return BadCRC;
	}

  }

  return Success;
}

int SelidarDriver::cacheScanData()
{
	SelidarMeasurementNode local_buf[128];
    size_t count = 128;
    SelidarMeasurementNode local_scan[MAX_SCAN_NODES];
    size_t scan_count = 0;
    int ans;
    memset(local_scan, 0, sizeof(local_scan));

    while(scanning)
    {
    	unsigned short range;
        if (IS_FAIL(ans = waitScanData(range, local_buf, count))) {
            if (ans != Timeout) {
                scanning = false;
                return Timeout;
            }
        }
        rxtx_lock.lock();
        if(range == SELIDAR_START_RANGES)
        {
        	cached_scan_node_count = 0;
        }

        if(cached_scan_node_count >= 2048)
        {
        	scanning = false;
        	return Failure;
        }

        for(int i = 0; i < count; i++)
        {
        	cached_scan_node_buf[cached_scan_node_count++] = local_buf[i];
        }

        if(range == SELIDAR_END_RANGES)
		{
        	data_cond.set();
		}
        rxtx_lock.unlock();
    }
    scanning = false;
    return Success;
}

int SelidarDriver::waitScanData(unsigned short& angle_range, SelidarMeasurementNode* nodes, size_t& node_count, unsigned int timeout)
{
  int ans;

  unsigned char checksum = 0;

  // waiting for confirmation
  SelidarPacketHead response_header;
  if (IS_FAIL(ans = waitResponseHeader(&response_header, timeout))) {
    return ans;
  }

  // verify whether we got a correct header
  if (response_header.cmd.bits.dir != Lidar2Host) {
    return Invalid;
  }

  if (response_header.cmd.bits.cmd_word != StartScanRep) {
    return Invalid;
  }

  for (size_t i = 0; i < sizeof(SelidarPacketHead); i++)
  {
    checksum ^= ((unsigned char*)&response_header + i);
  }

  //discard first packet
  size_t data_size = response_header.length - sizeof(SelidarPacketHead);

  if (rxtx->waitfordata(data_size, timeout) != Serial::ANS_OK) {
    return Timeout;
  }

  unsigned char scan_data[1024] = {0};

  rxtx->recvdata(scan_data, data_size);


  for (size_t i = 0; i < data_size - 1; i++)
  {
    checksum ^= (scan_data + i);
  }

  if(checksum != scan_data[data_size - 1])
  {
    return BadCRC;
  }

  int data_pos = 0;

  memcpy(&angle_range, scan_data, sizeof(angle_range));
  data_pos += sizeof(angle_range);

  node_count = (data_size - 2 - 2 - 1) / 2;

  unsigned short start_angle;
  memcpy(&start_angle, scan_data + data_pos, sizeof(start_angle));
  data_pos += sizeof(start_angle);

  for(size_t i = 0; i < node_count; i++)
  {
    unsigned short angle, distance;

    memcpy(&angle, scan_data + data_pos, sizeof(angle));
    data_pos += sizeof(angle);

    memcpy(&distance, scan_data + data_pos, sizeof(distance));
    data_pos += sizeof(distance);

    nodes[i].angle_scale_100 = start_angle + (i * angle_range) / node_count;
    nodes[i].distance_scale_1000 = distance;
  }

  return Success;
}

int SelidarDriver::startScan(unsigned int timeout)
{
  int ans;
  if (!connected)
    return Failure;
  if (scanning)
    return Denied;

  stop();

  // have to slow down the speed of sending cmd, otherwise next cmd will be discard by radar
  NS_NaviCommon::delay(100);

  {
    boost::mutex::scoped_lock auto_lock(rxtx_lock);

    if (IS_FAIL(ans = sendCommand(StartScanReq))) {
      return ans;
    }

    scanning = true;
    cache_thread = boost::thread(boost::bind(&SelidarDriver::cacheScanData, this));
  }

  return Success;
}


int SelidarDriver::grabScanData(SelidarMeasurementNode * nodebuffer, size_t & count, unsigned int timeout)
{
    switch (data_cond.wait(timeout/1000))
    {
    case NS_NaviCommon::Condition::COND_TIMEOUT:
        count = 0;
        return Timeout;
    case NS_NaviCommon::Condition::COND_OK:
        {
            if(cached_scan_node_count == 0) return Timeout; //consider as timeout

            boost::mutex::scoped_lock auto_lock(rxtx_lock);

            size_t size_to_copy = min(count, cached_scan_node_count);

            memcpy(nodebuffer, cached_scan_node_count, size_to_copy*sizeof(SelidarMeasurementNode));
            count = size_to_copy;
            cached_scan_node_count = 0;
        }
        return Success;

    default:
        count = 0;
        return Failure;
    }
}

}
