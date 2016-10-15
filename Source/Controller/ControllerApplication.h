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

#include "Communication/CommBase.h"
#include "Communication/SerialComm.h"
#include "Communication/SpiComm.h"
#include "../Application/Application.h"

namespace NS_Controller {

class ControllerApplication: public Application {
public:
  ControllerApplication();
  virtual ~ControllerApplication();
private:
  NS_DataType::Odometry current_odom;
  boost::thread controller_loop_thread;
  CommBase* communication;
private:
  void odomService(NS_ServiceType::RequestBase* request, NS_ServiceType::ResponseBase* response);
  void twistCallback(NS_DataType::DataBase* twist_data);
  void controllerLoop();
  void loadParameters();
public:
  virtual void initialize();
  virtual void run();
  virtual void quit();
};

} /* namespace NS_Controller */

#endif /* CONTROLLER_COTROLLERAPPLICATION_H_ */
