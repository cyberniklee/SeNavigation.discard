#ifndef _SELIDAR_DRIVER_H_
#define _SELIDAR_DRIVER_H_

#include <Thread/Condition.h>
#include "SelidarTypes.h"
#include "Serial.h"
#include "SelidarProtocol.h"

namespace NS_Selidar
{
  
#define DEFAULT_TIMEOUT 2000
#define MAX_SCAN_NODES 2048
  
  class SelidarDriver
  {
  public:
    SelidarDriver ();
    virtual
    ~SelidarDriver ();

  public:
    int
    connect (const char * port_path, unsigned int baudrate, unsigned int flag);
    void
    disconnect ();
    bool
    isConnected ();

    int
    reset (unsigned int timeout = DEFAULT_TIMEOUT);
    int
    stop (unsigned int timeout = DEFAULT_TIMEOUT);

    int
    getHealth (SelidarHealth &health_info, unsigned int timeout =
    DEFAULT_TIMEOUT);
    int
    getDeviceInfo (SelidarInfo &info, unsigned int timeout =
    DEFAULT_TIMEOUT);

    int
    startScan (unsigned int timeout = DEFAULT_TIMEOUT);

    int
    grabScanData (SelidarMeasurementNode * nodebuffer, size_t & count,
                  unsigned int timeout = DEFAULT_TIMEOUT);

  protected:
    int
    sendCommand (unsigned char cmd);
    void
    disableDataGrabbing ();
    int
    waitResponseHeader (SelidarPacketHead* header, unsigned int timeout =
    DEFAULT_TIMEOUT);

    int
    waitScanData (unsigned short& angle_range, SelidarMeasurementNode* nodes,
                  size_t& node_count, unsigned int timeout = DEFAULT_TIMEOUT);

    int
    cacheScanData ();

    bool connected;
    bool scanning;

    boost::mutex rxtx_lock;
    NS_NaviCommon::Condition data_cond;
    Serial* rxtx;

    SelidarMeasurementNode cached_scan_node_buf[2048];
    size_t cached_scan_node_count;

    boost::thread cache_thread;
  };

}

#endif
