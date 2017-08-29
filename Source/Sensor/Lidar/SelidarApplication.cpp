/*
 * SelidarApplication.cpp
 *
 *  Created on: 2016年10月12日
 *      Author: lichq
 */

#include "SelidarApplication.h"
#include <Console/Console.h>
//for debugging
#include <assert.h>
#include <Time/Utils.h>

namespace NS_Selidar
{
  
#define DEG2RAD(x) ((x)*M_PI/180.)
#define RAD2DEG(x) ((x)*180./M_PI)
  
#ifndef _countof
#define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))
#endif
  
  SelidarApplication::SelidarApplication ()
  {
    serial_baudrate = 115200;
    frame_id = "laser_frame";
    scan_count = 0;
    scan_timeout = 3;
    inverted = false;
  }
  
  SelidarApplication::~SelidarApplication ()
  {
    scan_count = 0;
  }
  
  void
  SelidarApplication::loadParameters ()
  {
    parameter.loadConfigurationFile ("selidar.xml");
    serial_port = parameter.getParameter ("serial_port", "/dev/ttyUSB0");
    serial_baudrate = parameter.getParameter ("serial_baudrate", 115200);
    frame_id = parameter.getParameter ("frame_id", "laser_frame");

    if (parameter.getParameter ("inverted", 0) == 1)
    {
      inverted = true;
    }
    else
    {
      inverted = false;
    }
  }
  
#ifdef DUPLEX_MODE
  bool
  SelidarApplication::checkSelidarHealth (SelidarDriver * drv)
  { 
    int op_result;
    SelidarHealth healthinfo;

    op_result = drv->getHealth (healthinfo);

    if (IS_OK(op_result))
    { 
      NS_NaviCommon::console.debug ("Selidar health status : %d, errcode: %d",
          healthinfo.status, healthinfo.err_code);

      if (healthinfo.status != StatusFine)
      { 
        NS_NaviCommon::console.warning ("Selidar's status is not fine! ");
        return false;
      }
      else
      { 
        NS_NaviCommon::console.message ("Selidar's status is not fine! ");
        return true;
      }

    }
    else
    { 
      return false;
    }
  }

  bool
  SelidarApplication::checkSelidarInfo (SelidarDriver * drv)
  { 
    int op_result;
    SelidarInfo device_info;

    op_result = drv->getDeviceInfo (device_info);

    if (IS_OK(op_result))
    { 
      NS_NaviCommon::console.debug ("Selidar device info :");
      NS_NaviCommon::console.debug ("\t model : %d ", device_info.model);
      NS_NaviCommon::console.debug ("\t hw ver : %d ", device_info.hw_id);
      NS_NaviCommon::console.debug ("\t fw ver : %d.%d ", device_info.fw_major,
          device_info.fw_minor);
      return true;
    }
    else
    { 
      return false;
    }

  }

  bool
  SelidarApplication::stopScanService (NS_ServiceType::RequestBase* request,
      NS_ServiceType::ResponseBase* response)
  { 
    if (!drv.isConnected ())
    return false;

    drv.stop ();

    return true;
  }

  bool
  SelidarApplication::startScanService (NS_ServiceType::RequestBase* request,
      NS_ServiceType::ResponseBase* response)
  { 
    if (!drv.isConnected ())
    return false;

    NS_NaviCommon::console.message ("Start motor");

    drv.startScan ();
    return true;
  }
#endif
  
  void
  SelidarApplication::publishScan (SelidarMeasurementNode *nodes,
                                   size_t node_count, NS_NaviCommon::Time start,
                                   double scan_time, float angle_min,
                                   float angle_max)
  {
    NS_DataType::LaserScan* scan_msg = new NS_DataType::LaserScan;
    
    scan_msg->header.stamp = start;
    scan_msg->header.frame_id = frame_id;

    scan_count_lock.lock ();
    scan_count++;
    if (scan_count == 1)
    {
      NS_NaviCommon::console.debug ("Got first scan data!");
      got_first_scan_cond.notify_one ();
    }
    scan_count_lock.unlock ();

    bool reversed = (angle_max > angle_min);
    if (reversed)
    {
      scan_msg->angle_min =  M_PI - angle_max;
      scan_msg->angle_max =  M_PI - angle_min;
    } else {
      scan_msg->angle_min =  M_PI - angle_min;
      scan_msg->angle_max =  M_PI - angle_max;
    }


    scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min)
        / (double) (node_count - 1);
    
    scan_msg->scan_time = scan_time;
    scan_msg->time_increment = scan_time / (double) (node_count - 1);
    scan_msg->range_min = 0.15f;
    scan_msg->range_max = 8.0f;
    
    scan_msg->intensities.resize (node_count);
    scan_msg->ranges.resize (node_count);

    bool reverse_data = (!inverted && reversed) || (inverted && !reversed);

    if (!reverse_data)
    {
      for (size_t i = 0; i < node_count; i++)
      {
        float read_value = (float) nodes[i].distance_scale_1000 / 1000.0f;
        if (read_value == 0.0)
          scan_msg->ranges[i] = std::numeric_limits<float>::infinity();
        else
          scan_msg->ranges[i] = read_value;
      }
    }
    else
    {
      for (size_t i = 0; i < node_count; i++)
      {
        float read_value = (float) nodes[i].distance_scale_1000 / 1000.0f;
        if (read_value == 0.0)
          scan_msg->ranges[node_count-1-i] = std::numeric_limits<float>::infinity();
        else
          scan_msg->ranges[node_count-1-i] = read_value;
      }
    }
    ///////////////////////////////////////////////////////////////////////////////////////
    /*
    for (size_t i = 0; i < node_count; i++)
    {
      float read_value = (float) nodes[i].distance_scale_1000 / 1000.0f;
      printf("-->%d    %f\n", i, read_value);
    }
    */
    //////////////////////////////////////////////////////////////////////////////////////

    dispitcher->publish (NS_NaviCommon::DATA_TYPE_LASER_SCAN, scan_msg);
  }
  
  void
  SelidarApplication::scanLoop ()
  {
    int op_result;
    drv.startScan ();
    NS_NaviCommon::Time start_scan_time;
    NS_NaviCommon::Time end_scan_time;
    double scan_duration;

    const int buffer_size = 360 * 4;
    SelidarMeasurementNode nodes_buffer[buffer_size];
    SelidarMeasurementNode nodes_temp[buffer_size];
    SelidarMeasurementNode nodes_pub[buffer_size];
    size_t buffered_nodes = 0;
    memset (nodes_buffer, 0, sizeof(nodes_buffer));
    memset (nodes_temp, 0, sizeof(nodes_temp));
    memset (nodes_pub, 0, sizeof(nodes_pub));

    while (running)
    {
      SelidarMeasurementNode nodes[buffer_size];
      size_t count = _countof(nodes);
      start_scan_time = NS_NaviCommon::Time::now ();
      op_result = drv.grabScanData (nodes, count);
      end_scan_time = NS_NaviCommon::Time::now ();
      scan_duration = (end_scan_time - start_scan_time).toSec () * 1e-3;

      if (op_result == Success)
      {
        if ((buffered_nodes + count) > buffer_size)
        {
          NS_NaviCommon::console.warning ("Lidar buffer is full!");
          buffered_nodes = 0;
          continue;
        }

        for (int i = 0; i < count; i++)
        {
          nodes_buffer[buffered_nodes] = nodes[i];
          if (nodes_buffer[buffered_nodes].angle_scale_100 >= 36000)
            nodes_buffer[buffered_nodes].angle_scale_100 -= 36000;
          buffered_nodes++;
        }

        int pub_nodes_count = 0;

        for (int i = 0; i < buffered_nodes; i++)
        {
          if (i > 0 && nodes_buffer[i].angle_scale_100 < nodes_buffer[i - 1].angle_scale_100)
          {
            pub_nodes_count = i;
            //copy publish nodes
            for (int j = 0; j < pub_nodes_count; j++)
            {
              nodes_pub[j] = nodes_buffer[j];
            }

            //re-serialize,save temp
            for (int j = 0; j < buffered_nodes; j++)
            {
              nodes_temp[j] = nodes_buffer[pub_nodes_count + j];
            }

            //save to buffer
            buffered_nodes -= pub_nodes_count;
            for (int j = 0; j < buffered_nodes; j++)
            {
              nodes_buffer[j] = nodes_temp[j];
            }

            break;
          }
        }
/*
        if (pub_nodes_count > 0)
        {

          float angle_min = (nodes_pub[0].angle_scale_100 % 36000) / 100.0f;
          float angle_max = (nodes_pub[pub_nodes_count - 1].angle_scale_100 % 36000) / 100.0f;


          angle_min = DEG2RAD(angle_min);
          angle_max = DEG2RAD(angle_max);

          publishScan (nodes_pub, pub_nodes_count, start_scan_time, scan_duration, angle_min, angle_max);
        }
*/
        if (pub_nodes_count > 0)
        {
          float angle_min = DEG2RAD(0.0f);
          float angle_max = DEG2RAD(359.0f);

          const int angle_compensate_nodes_count = 360;
          const int angle_compensate_multiple = 1;
          int angle_compensate_offset = 0;
          SelidarMeasurementNode angle_compensate_nodes[angle_compensate_nodes_count];
          memset (angle_compensate_nodes, 0, angle_compensate_nodes_count * sizeof(SelidarMeasurementNode));

          int i, j;
          for (i = 0; i < pub_nodes_count; i++)
          {
            if (nodes_pub[i].distance_scale_1000 != 0)
            {
              float angle = (float) (nodes_pub[i].angle_scale_100) / 100.0f;
              int angle_value = (int) (angle * angle_compensate_multiple);
              if ((angle_value - angle_compensate_offset) < 0)
                angle_compensate_offset = angle_value;

              for (j = 0; j < angle_compensate_multiple; j++)
              {
                angle_compensate_nodes[angle_value - angle_compensate_offset + j] = nodes[i];
              }
            }
          }
          publishScan (angle_compensate_nodes, angle_compensate_nodes_count,
                       start_scan_time, scan_duration, angle_min, angle_max);
        }


        /*
        float angle_min = DEG2RAD(0.0f);
        float angle_max = DEG2RAD(359.0f);
        
        const int angle_compensate_nodes_count = 360;
        const int angle_compensate_multiple = 1;
        int angle_compensate_offset = 0;
        SelidarMeasurementNode angle_compensate_nodes[angle_compensate_nodes_count];
        memset (angle_compensate_nodes, 0, angle_compensate_nodes_count * sizeof(SelidarMeasurementNode));

        int i, j;
        for (i = 0; i < count; i++)
        {
          if (nodes[i].distance_scale_1000 != 0)
          {
            float angle = (float) (nodes[i].angle_scale_100) / 100.0f;
            int angle_value = (int) (angle * angle_compensate_multiple);
            if ((angle_value - angle_compensate_offset) < 0)
              angle_compensate_offset = angle_value;

            for (j = 0; j < angle_compensate_multiple; j++)
            {
              angle_compensate_nodes[angle_value - angle_compensate_offset + j] = nodes[i];
            }
          }
        }
        publishScan (angle_compensate_nodes, angle_compensate_nodes_count,
                     start_scan_time, scan_duration, angle_min, angle_max);
        */


        
      }
    }
  }
  
  void
  SelidarApplication::initialize ()
  {
    NS_NaviCommon::console.message ("selidar is initializing!");
    
    loadParameters ();
    
    // make connection...
    if (IS_FAIL(
        drv.connect (serial_port.c_str (), (unsigned int )serial_baudrate, 0)))
    {
      NS_NaviCommon::console.error (
          "cannot bind to the specified serial port %s.", serial_port.c_str ());
    }
    
#ifdef DUPLEX_MODE
    // reset lidar
    drv.reset ();
    NS_NaviCommon::delay (5000);

    // check health...
    if (!checkSelidarHealth (&drv))
    { 
      return;
    }

    NS_NaviCommon::delay (100);

    // get device info...
    if (!checkSelidarInfo (&drv))
    { 
      return;
    }
    NS_NaviCommon::delay (100);

    service->advertise (
        NS_NaviCommon::SERVICE_TYPE_STOP_SCAN,
        boost::bind (&SelidarApplication::stopScanService, this, _1, _2));
    service->advertise (
        NS_NaviCommon::SERVICE_TYPE_START_SCAN,
        boost::bind (&SelidarApplication::startScanService, this, _1, _2));
#endif
    
    NS_NaviCommon::delay (100);
    
    initialized = true;
    
    NS_NaviCommon::console.message ("selidar has initialized!");
  }
  
  void
  SelidarApplication::run ()
  {
    NS_NaviCommon::console.message ("selidar is running!");
    
    running = true;
    
    scan_thread = boost::thread (
        boost::bind (&SelidarApplication::scanLoop, this));

    int wait_times = 0;
    scan_count_lock.lock ();
    while (wait_times++ <= scan_timeout && scan_count == 0)
    {
      got_first_scan_cond.timed_wait (scan_count_lock,
                                      (boost::get_system_time () + boost::posix_time::seconds (1)));
    }
    scan_count_lock.unlock ();

    if (scan_count == 0)
    {
      NS_NaviCommon::console.error ("Can't got first scan from LIDAR.");
      running = false;
    }

  }
  
  void
  SelidarApplication::quit ()
  {
    NS_NaviCommon::console.message ("selidar is quitting!");
    
#ifdef DUPLEX_MODE
    drv.stop ();
#endif
    
    running = false;
    
    scan_thread.join ();
    
    drv.disconnect ();
  }

}

