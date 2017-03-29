/*
 * ControllerApplication.h
 *
 *  Created on: 2016年10月15日
 *      Author: seeing
 */

#ifndef _CONTROLLERAPPLICATION_H_
#define _CONTROLLERAPPLICATION_H_

#include <DataSet/DataType/DataBase.h>
#include <Service/ServiceType/RequestBase.h>
#include <Service/ServiceType/ResponseBase.h>
#include <Transform/LinearMath/Transform.h>
#include <DataSet/DataType/Odometry.h>
#include <DataSet/DataType/Twist.h>
#include <Time/Time.h>
#include <Time/Duration.h>

#include <boost/thread/thread.hpp>

#include "Communication/SpiComm.h"
#include "../Application/Application.h"

namespace NS_Controller
{
  
  typedef struct
  {
    double x;
    double y;
    double theta;
  } PoseState;
  
  class ControllerApplication: public Application
  {
  public:
    ControllerApplication ();
    virtual
    ~ControllerApplication ();
  private:
    boost::thread controller_loop_thread;
    SpiComm* comm;

    boost::mutex odom_lock;

    PoseState current_pose;
    NS_DataType::Odometry current_odometry;
  private:
    std::string comm_dev_name_;

    double control_rate_;
    double control_timeout_;
    double wheel_diameter_;
    int encoder_resolution_;
    int gear_reduction_;
    double wheel_track_;
    double accel_limit_;

    /*
     * PID parameters
     */
    int pid_kp_;
    int pid_kd_;
    int pid_ki_;
    int pid_ko_;

    double ticks_per_meter;
    double distance_per_tick;
  private:
    
    void
    odomService (NS_ServiceType::RequestBase* request,
                 NS_ServiceType::ResponseBase* response);
    void
    odomTransformService (NS_ServiceType::RequestBase* request,
                          NS_ServiceType::ResponseBase* response);
    void
    twistCallback (NS_DataType::DataBase* twist_data);
    void
    loadParameters ();
    void
    configController ();

  public:
    virtual void
    initialize ();
    virtual void
    run ();
    virtual void
    quit ();
  };

} /* namespace NS_Controller */

#endif /* CONTROLLER_COTROLLERAPPLICATION_H_ */
