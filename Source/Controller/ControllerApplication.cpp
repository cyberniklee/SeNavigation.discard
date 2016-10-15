/*
 * ControllerApplication.cpp
 *
 *  Created on: 2016年10月15日
 *      Author: seeing
 */

#include "ControllerApplication.h"
#include <Service/ServiceType/RequestOdometry.h>
#include <Service/ServiceType/ResponseOdometry.h>
#include <DataSet/Dispitcher.h>
#include <Console/Console.h>
#include <Service/Service.h>

namespace NS_Controller {

ControllerApplication::ControllerApplication() {
	// TODO Auto-generated constructor stub

}

ControllerApplication::~ControllerApplication() {
	// TODO Auto-generated destructor stub
}

void ControllerApplication::odomService(NS_ServiceType::RequestBase* request, NS_ServiceType::ResponseBase* response)
{
  NS_ServiceType::RequestOdometry* req = (NS_ServiceType::RequestOdometry*)request;
  NS_ServiceType::ResponseOdometry* rep = (NS_ServiceType::ResponseOdometry*)response;

}

void ControllerApplication::twistCallback(NS_DataType::DataBase* twist_data)
{

}

void ControllerApplication::controllerLoop()
{
  while(running)
  {

  }
}

void ControllerApplication::loadParameters()
{

}

void ControllerApplication::initialize()
{
  NS_NaviCommon::console.message("controller is initializing!");
  loadParameters();

  dispitcher->subscribe(NS_NaviCommon::DATA_TYPE_TWIST,
  		  boost::bind(&ControllerApplication::twistCallback, this, _1));

  service->advertise(NS_NaviCommon::SERVICE_TYPE_RAW_ODOMETRY,
  		  boost::bind(&ControllerApplication::odomService, this, _1, _2));

  controller_loop_thread = boost::thread(boost::bind(&ControllerApplication::controllerLoop, this));

  initialized = true;
}

void ControllerApplication::run()
{
  NS_NaviCommon::console.message("controller is running!");
  running = true;
  controller_loop_thread.join();
}

void ControllerApplication::quit()
{
  NS_NaviCommon::console.message("controller is quitting!");
  running = false;
}

} /* namespace NS_Controller */
