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
  memset(&current_odom_state, 0, sizeof(current_odom_state));
  memset(&last_odom_state, 0, sizeof(last_odom_state));
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
  rep->odom = current_odometry;

}

void ControllerApplication::odomTransformService(NS_ServiceType::RequestBase* request, NS_ServiceType::ResponseBase* response)
{
  NS_ServiceType::RequestTransform* req = (NS_ServiceType::RequestTransform*)request;
  NS_ServiceType::ResponseTransform* rep = (NS_ServiceType::ResponseTransform*)response;

  boost::mutex::scoped_lock locker_(odom_lock);

  NS_NaviCommon::console.debug("odometry pose: x:%f, y:%f, theta:%f.",
		  current_pose.x, current_pose.y, current_pose.theta);

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

  double x = twist->linear.x;
  double theta = twist->angular.z;
}

bool ControllerApplication::calcOdom(OdometryState& cur_odom, OdometryState& last_odom,
		  PoseState& cur_pose, NS_DataType::Odometry& cur_odometry)
{
  boost::mutex::scoped_lock locker_(odom_lock);

  NS_NaviCommon::Time now = NS_NaviCommon::Time::now();

  long _seconds = cur_odom.time_stamp - last_odom.time_stamp;
  int _m_seconds = cur_odom.time_stamp_ms - last_odom.time_stamp_ms;
  long dt_by_ms = _seconds * 1000 + _m_seconds;

  double dright = (cur_odom.right_enc - last_odom.right_enc) / ticks_per_meter;
  double dleft = (cur_odom.left_enc - last_odom.left_enc) / ticks_per_meter;
/*
  printf("%ld, ticks_per_meter=%f\n", dt_by_ms, ticks_per_meter);
  printf("<+>%ld, %ld\n", cur_odom.right_enc, cur_odom.left_enc);
  printf("<->%ld, %ld\n", last_odom.right_enc, last_odom.left_enc);
  printf("<.>%f, %f\n", dright, dleft);
*/
  double dxy_ave = (dright + dleft) / 2.0;
  double dth = (dright - dleft) / wheel_track_;

  double vxy = 0.0;
  double vth = 0.0;
  double dx = 0.0, dy = 0.0;

  if(dt_by_ms == 0)
  {
    last_odom = cur_odom;
    return false;
  }

  vxy = (dxy_ave / dt_by_ms) / 1000;
  vth = (dth / dt_by_ms) / 1000;

  if(dxy_ave != 0)
  {
    dx = cos(dth) * dxy_ave;
    dy = -sin(dth) * dxy_ave;
    cur_pose.x += (cos(cur_pose.theta) * dx - sin(cur_pose.theta) * dy);
    cur_pose.y += (sin(cur_pose.theta) * dx + cos(cur_pose.theta) * dy);
  }

  if(dth != 0)
  {
    cur_pose.theta += dth;
  }

  cur_odometry.header.stamp = now;
  cur_odometry.pose.position.x = cur_pose.x;
  cur_odometry.pose.position.y = cur_pose.y;
  cur_odometry.pose.orientation.x = 0.0;
  cur_odometry.pose.orientation.y = 0.0;
  cur_odometry.pose.orientation.z = sin(cur_pose.theta / 2.0);
  cur_odometry.pose.orientation.w = cos(cur_pose.theta / 2.0);
  cur_odometry.twist.linear.x = vxy;
  cur_odometry.twist.linear.y = 0;
  cur_odometry.twist.angular.z = vth;

  last_odom = cur_odom;

  return true;
}

void ControllerApplication::controllerLoop()
{
  NS_NaviCommon::Rate rate(10);
  while(running)
  {
    //TODO:for test
    /*
    current_odom_state.left_enc += 100;
    current_odom_state.right_enc += 10;
    current_odom_state.time_stamp_ms += 100;
    if(current_odom_state.time_stamp_ms == 1000)
    {
      current_odom_state.time_stamp++;
      current_odom_state.time_stamp_ms = 0;
    }
    */
    calcOdom(current_odom_state, last_odom_state, current_pose, current_odometry);

    rate.sleep();
  }
}

void ControllerApplication::loadParameters()
{
  parameter.loadConfigurationFile("controller.xml");
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

  controller_loop_thread = boost::thread(boost::bind(&ControllerApplication::controllerLoop, this));
}

void ControllerApplication::quit()
{
  NS_NaviCommon::console.message("controller is quitting!");
  running = false;
  controller_loop_thread.join();
}

} /* namespace NS_Controller */
