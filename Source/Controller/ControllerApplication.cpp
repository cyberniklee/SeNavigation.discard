/*
 * ControllerApplication.cpp
 *
 *  Created on: 2016年10月15日
 *      Author: seeing
 */

#include "ControllerApplication.h"
#include <Service/ServiceType/RequestOdometry.h>
#include <Service/ServiceType/ResponseOdometry.h>
#include <Service/ServiceType/RequestTransform.h>
#include <Service/ServiceType/ResponseTransform.h>
#include <DataSet/Dispitcher.h>
#include <Console/Console.h>
#include <Service/Service.h>
#include <Transform/LinearMath/Quaternion.h>
#include <Transform/LinearMath/Vector3.h>
#include <Time/Rate.h>

namespace NS_Controller {

ControllerApplication::ControllerApplication() {
  memset(&current_pose, 0, sizeof(current_pose));
  memset(&current_odometry, 0, sizeof(current_odometry));
}

ControllerApplication::~ControllerApplication() {
	// TODO Auto-generated destructor stub
}

void ControllerApplication::odomService(NS_ServiceType::RequestBase* request, NS_ServiceType::ResponseBase* response)
{
  NS_ServiceType::RequestOdometry* req = (NS_ServiceType::RequestOdometry*)request;
  NS_ServiceType::ResponseOdometry* rep = (NS_ServiceType::ResponseOdometry*)response;

  boost::mutex::scoped_lock locker_(odom_lock);

  float x = comm->getFloatValue(BASE_REG_ODOM_X);
  float y = comm->getFloatValue(BASE_REG_ODOM_Y);
  float theta = comm->getFloatValue(BASE_REG_ODOM_THETA);
  float v_xy = comm->getFloatValue(BASE_REG_ODOM_LINEAR_SPD);
  float v_th = comm->getFloatValue(BASE_REG_ODOM_ANGULAR_SPD);

  current_odometry.pose.position.x = x;
  current_odometry.pose.position.y = y;
  current_odometry.pose.orientation.x = 0.0f;
  current_odometry.pose.orientation.y = 0.0f;
  current_odometry.pose.orientation.z = sin(theta / 2.0);
  current_odometry.pose.orientation.w = cos(theta / 2.0);
  current_odometry.twist.linear.x = v_xy;
  current_odometry.twist.linear.y = 0.0f;
  current_odometry.twist.angular.z = v_th;

  rep->odom = current_odometry;

}

void ControllerApplication::odomTransformService(NS_ServiceType::RequestBase* request, NS_ServiceType::ResponseBase* response)
{
  NS_ServiceType::RequestTransform* req = (NS_ServiceType::RequestTransform*)request;
  NS_ServiceType::ResponseTransform* rep = (NS_ServiceType::ResponseTransform*)response;

  boost::mutex::scoped_lock locker_(odom_lock);

  NS_NaviCommon::console.debug("odometry pose: x:%f, y:%f, theta:%f.",
		  current_pose.x, current_pose.y, current_pose.theta);

  current_pose.x = comm->getFloatValue(BASE_REG_ODOM_X);
  current_pose.y = comm->getFloatValue(BASE_REG_ODOM_Y);
  current_pose.theta = comm->getFloatValue(BASE_REG_ODOM_THETA);

  rep->transform.translation.x = current_pose.x;
  rep->transform.translation.y = current_pose.y;
  rep->transform.translation.z = 0.0;

  rep->transform.rotation.x = 0.0;
  rep->transform.rotation.y = 0.0;
  rep->transform.rotation.z = sin(current_pose.theta / 2.0);
  rep->transform.rotation.w = cos(current_pose.theta / 2.0);

  NS_NaviCommon::console.debug("odometry transform: x:%f, y:%f, rz:%f, rw:%f",
		  rep->transform.translation.x, rep->transform.translation.y,
		  rep->transform.rotation.z, rep->transform.rotation.w);
}

void ControllerApplication::twistCallback(NS_DataType::DataBase* twist_data)
{
  NS_DataType::Twist* twist = (NS_DataType::Twist*)twist_data;

  float x = twist->linear.x;
  float theta = twist->angular.z;

  comm->setFloatValue(BASE_REG_LINEAR_SPD, x);
  comm->setFloatValue(BASE_REG_ANGULAR_SPD, theta);
}

void ControllerApplication::loadParameters()
{
  parameter.loadConfigurationFile("controller.xml");
  comm_dev_name_ = parameter.getParameter("device", "/dev/seeing-stm32");
  control_rate_ = parameter.getParameter("control_rate", 10.0f);
  control_timeout_ = parameter.getParameter("control_timeout", 1.0f);
  wheel_diameter_ = parameter.getParameter("wheel_diameter", 0.068f);
  encoder_resolution_ = parameter.getParameter("encoder_resolution", 16);
  gear_reduction_ = parameter.getParameter("gear_reduction", 62);

  wheel_track_ = parameter.getParameter("wheel_track", 0.265f);
  accel_limit_ = parameter.getParameter("accel_limit", 1.0f);

  pid_kp_ = parameter.getParameter("pid_kp", 20);
  pid_kd_ = parameter.getParameter("pid_kd", 12);
  pid_ki_ = parameter.getParameter("pid_ki", 0);
  pid_ko_ = parameter.getParameter("pid_ko", 50);

  ticks_per_meter = (encoder_resolution_ * gear_reduction_) / (wheel_diameter_ * M_PI);
}

void ControllerApplication::initialize()
{
  NS_NaviCommon::console.message("controller is initializing!");
  loadParameters();

  comm = new SpiComm(comm_dev_name_);
  if(!comm->open())
  {
    NS_NaviCommon::console.error("can't open base controller device!");
    return;
  }

  dispitcher->subscribe(NS_NaviCommon::DATA_TYPE_TWIST,
  		  boost::bind(&ControllerApplication::twistCallback, this, _1));

  service->advertise(NS_NaviCommon::SERVICE_TYPE_RAW_ODOMETRY,
  		  boost::bind(&ControllerApplication::odomService, this, _1, _2));

  service->advertise(NS_NaviCommon::SERVICE_TYPE_ODOMETRY_BASE_TRANSFORM,
    	  boost::bind(&ControllerApplication::odomTransformService, this, _1, _2));

  initialized = true;
}

void ControllerApplication::run()
{
  NS_NaviCommon::console.message("controller is running!");

  running = true;
}

void ControllerApplication::quit()
{
  NS_NaviCommon::console.message("controller is quitting!");
  running = false;
  controller_loop_thread.join();
}

} /* namespace NS_Controller */
