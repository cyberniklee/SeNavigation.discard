#include "OdometryHelper.h"

namespace NS_Planner {

  OdometryHelper::OdometryHelper(NS_NaviCommon::Service* service) {
    service_ = service;
}

void OdometryHelper::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_INFO_ONCE("odom received!");

  //we assume that the odometry is published in the frame of the base
  boost::mutex::scoped_lock lock(odom_mutex_);
  base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
  base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
  base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
  base_odom_.child_frame_id = msg->child_frame_id;
//  ROS_DEBUG_NAMED("dwa_local_planner", "In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
//      base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z);
}

//copy over the odometry information
void OdometryHelperRos::getOdom(nav_msgs::Odometry& base_odom) {
  boost::mutex::scoped_lock lock(odom_mutex_);
  base_odom = base_odom_;
}


void OdometryHelperRos::getRobotVel(tf::Stamped<tf::Pose>& robot_vel) {
  // Set current velocities from odometry
  geometry_msgs::Twist global_vel;
  {
    boost::mutex::scoped_lock lock(odom_mutex_);
    global_vel.linear.x = base_odom_.twist.twist.linear.x;
    global_vel.linear.y = base_odom_.twist.twist.linear.y;
    global_vel.angular.z = base_odom_.twist.twist.angular.z;

    robot_vel.frame_id_ = base_odom_.child_frame_id;
  }
  robot_vel.setData(tf::Transform(tf::createQuaternionFromYaw(global_vel.angular.z), tf::Vector3(global_vel.linear.x, global_vel.linear.y, 0)));
  robot_vel.stamp_ = ros::Time();
}

void OdometryHelperRos::setOdomTopic(std::string odom_topic)
{
  if( odom_topic != odom_topic_ )
  {
    odom_topic_ = odom_topic;

    if( odom_topic_ != "" )
    {
      ros::NodeHandle gn;
      odom_sub_ = gn.subscribe<nav_msgs::Odometry>( odom_topic_, 1, boost::bind( &OdometryHelperRos::odomCallback, this, _1 ));
    }
    else
    {
      odom_sub_.shutdown();
    }
  }
}

} /* namespace base_local_planner */
