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

namespace NS_Controller {

typedef struct{
  long left_enc;
  long right_enc;
  long time_stamp;
  long time_stamp_ms;
}OdometryState;

typedef struct{
  double x;
  double y;
  double theta;
}PoseState;

class ControllerApplication: public Application {
public:
  ControllerApplication();
  virtual ~ControllerApplication();
private:
  boost::thread controller_loop_thread;
  //CommBase* communication;

  boost::mutex odom_lock;
  OdometryState current_odom_state;
  OdometryState last_odom_state;

  PoseState current_pose;
  NS_DataType::Odometry current_odometry;
private:
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
private:

  void odomService(NS_ServiceType::RequestBase* request, NS_ServiceType::ResponseBase* response);
  void odomTransformService(NS_ServiceType::RequestBase* request, NS_ServiceType::ResponseBase* response);
  void twistCallback(NS_DataType::DataBase* twist_data);
  void controllerLoop();
  void loadParameters();

  bool calcOdom(OdometryState& cur_odom, OdometryState& last_odom,
		  PoseState& cur_pose, NS_DataType::Odometry& cur_odometry);
public:
  virtual void initialize();
  virtual void run();
  virtual void quit();
};

} /* namespace NS_Controller */

#endif /* CONTROLLER_COTROLLERAPPLICATION_H_ */
