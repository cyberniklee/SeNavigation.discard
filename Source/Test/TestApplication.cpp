/*
 * CommunicatiorApplication.cpp
 *
 *  Created on: 2016年10月31日
 *      Author: nico
 */

#include <Console/Console.h>
#include <Service/Service.h>
#include <vector>
#include <stdlib.h>
#include <DataSet/DataType/PoseStamped.h>
#include <Transform/DataTypes.h>
#include "TestApplication.h"
#include <MapGenerator/MapGenerator.h>
#include <Service/ServiceType/RequestTransform.h>
#include <Service/ServiceType/ResponseTransform.h>

namespace NS_Test
{
  
  TestApplication::TestApplication ()
  {

  }
  
  TestApplication::~TestApplication ()
  {
  }
  
  void
  TestApplication::loadParameters ()
  {
    parameter.loadConfigurationFile ("test.xml");
    map_file_ = parameter.getParameter ("map_file", "/tmp/gmap.pgm");
    map_gen_freq_ = parameter.getParameter ("map_gen_freq", 1.0f);
  }
  
  bool
  TestApplication::getRobotPose (
      NS_Transform::Stamped<NS_Transform::Pose>& global_pose)
  {
    NS_ServiceType::RequestTransform request_odom;
    NS_ServiceType::ResponseTransform odom_transform;
    NS_ServiceType::ResponseTransform map_transform;

    if (service->call (NS_NaviCommon::SERVICE_TYPE_ODOMETRY_BASE_TRANSFORM,
                       &request_odom, &odom_transform) == false)
    {
      NS_NaviCommon::console.warning ("Get odometry transform failure!");
      return false;
    }

    if (service->call (NS_NaviCommon::SERVICE_TYPE_MAP_ODOMETRY_TRANSFORM,
                       &request_odom, &map_transform) == false)
    {
      NS_NaviCommon::console.warning ("Get map transform failure!");
      return false;
    }

    //TODO: not verify code for transform
    NS_Transform::Transform odom_tf, map_tf;
    NS_Transform::transformMsgToTF (odom_transform.transform, odom_tf);
    NS_Transform::transformMsgToTF (map_transform.transform, map_tf);

    global_pose.setData (odom_tf * map_tf);

    return true;
  }

  void
  TestApplication::mapGenerateLoop (double frequency)
  {
    NS_NaviCommon::Rate rate (frequency);
    int file_no = 0;
    while (running)
    {
      NS_ServiceType::ResponseMap map_resp;

      service->call (SERVICE_TYPE_MAP, NULL, &map_resp);

      if (map_resp.result)
      {
        ////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////   test    ////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////
        NS_Transform::Stamped<NS_Transform::Pose> pose;
        NS_DataType::PoseStamped pose_data;

        if (getRobotPose (pose))
        {
          NS_Transform::poseStampedTFToMsg(pose, pose_data);

          NS_NaviCommon::console.debug ("get robot pose %.3f, %.3f", (pose_data.pose.position.x * map_resp.map.info.resolution), (pose_data.pose.position.y * map_resp.map.info.resolution));

          NS_NaviCommon::MapGenerator::addRobotPoseInMap (map_resp.map.data, map_resp.map.info.height, map_resp.map.info.width, pose_data.pose.position.x, pose_data.pose.position.y);
        }

        NS_NaviCommon::MapGenerator::saveMapInPGM (map_resp.map.data, map_resp.map.info.height, map_resp.map.info.width, map_file_);
        file_no++;
        NS_NaviCommon::console.message ("Save map file No: %d!", file_no);
      }
      rate.sleep ();
    }
  }

  void
  TestApplication::initialize ()
  {
    NS_NaviCommon::console.message ("Test application is initializing!");
    
    loadParameters ();
    
    initialized = true;
  }
  
  void
  TestApplication::run ()
  {
    NS_NaviCommon::console.message ("Test application is running!");
    Application::running = true;
    
    map_gen_thread = boost::thread (
        boost::bind (&TestApplication::mapGenerateLoop, this, map_gen_freq_));
  }
  
  void
  TestApplication::quit ()
  {
    NS_NaviCommon::console.message ("Test application is quitting!");
    
    Application::running = false;

    map_gen_thread.join ();
  }
}

