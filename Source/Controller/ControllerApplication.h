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

#include "Simulator/SimulateController.h"

#define USE_SIMULATOR

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
    SpiComm* comm;

    boost::mutex base_lock;

    PoseState current_pose;
    NS_DataType::Odometry current_odometry;
  private:
    std::string comm_dev_name_;

    double wheel_diameter_;
    int encoder_resolution_;
    int gear_reduction_;
    double wheel_track_;
    double accel_limit_;

    int control_timeout_;

    /*
     * PID parameters
     */
    double pid_kp_right_;
    double pid_kd_right_;
    double pid_ki_right_;
    double pid_ko_right_;
    double pid_max_right_;
    double pid_min_right_;

    double pid_kp_left_;
    double pid_kd_left_;
    double pid_ki_left_;
    double pid_ko_left_;
    double pid_max_left_;
    double pid_min_left_;

    int control_duration_;

#ifdef USE_SIMULATOR
    SimulateController simulator;
#endif
    
  private:
    
    void
    odomService (NS_ServiceType::RequestBase* request,
                 NS_ServiceType::ResponseBase* response);
    void
    odomTransformService (NS_ServiceType::RequestBase* request,
                          NS_ServiceType::ResponseBase* response);

    void
    velocityCallback (NS_DataType::DataBase* velocity);

    void
    loadParameters ();
    void
    configController ();

    bool
    checkDevice ();

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
