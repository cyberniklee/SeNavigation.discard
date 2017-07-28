/*
 * SelidarApplication.h
 *
 *  Created on: 2016年10月12日
 *      Author: lichq
 */

#ifndef _SELIDARAPPLICATION_H_
#define _SELIDARAPPLICATION_H_

#include "../../Application/Application.h"
#include <DataSet/DataType/LaserScan.h>
#include <DataSet/DataType/DataBase.h>
#include <Service/ServiceType/RequestBase.h>
#include <Service/ServiceType/ResponseBase.h>
#include <Time/Time.h>
#include "Driver/SelidarDriver.h"
#include <boost/thread/thread.hpp>

//#define DUPLEX_MODE

namespace NS_Selidar
{
  class SelidarApplication: public Application
  {
  public:
    SelidarApplication ();
    ~SelidarApplication ();
  private:
    std::string serial_port;
    int serial_baudrate;
    std::string frame_id;
    bool inverted;
    bool angle_compensate;
    int scan_timeout;

    SelidarDriver drv;
    boost::thread scan_thread;

    int scan_count;

    boost::condition got_first_scan_cond;
    boost::mutex scan_count_lock;
  private:
    
#ifdef DUPLEX_MODE
    bool
    checkSelidarHealth (SelidarDriver * drv);
    bool
    checkSelidarInfo (SelidarDriver * drv);
    bool
    startScanService (NS_ServiceType::RequestBase* request,
        NS_ServiceType::ResponseBase* response);
    bool
    stopScanService (NS_ServiceType::RequestBase* request,
        NS_ServiceType::ResponseBase* response);
#endif
    
    void
    loadParameters ();

    void
    publishScan (SelidarMeasurementNode *nodes, size_t node_count,
                 NS_NaviCommon::Time start, double scan_time, float angle_min,
                 float angle_max);
    void
    scanLoop ();

  public:
    virtual void
    initialize ();
    virtual void
    run ();
    virtual void
    quit ();
  };
}

#endif /* SELIDARAPPLICATION_H_ */
