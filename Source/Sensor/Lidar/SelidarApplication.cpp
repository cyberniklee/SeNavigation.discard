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

namespace NS_Selidar {

#define DEG2RAD(x) ((x)*M_PI/180.)
#define RAD2DEG(x) ((x)*180./M_PI)

#ifndef _countof
#define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))
#endif

SelidarApplication::SelidarApplication() {
	serial_baudrate = 115200;
    inverted = false;
	angle_compensate = true;
    frame_id = "laser_frame";
}

SelidarApplication::~SelidarApplication() {

}

void SelidarApplication::loadParameters() {
	parameter.loadConfigurationFile("selidar.xml");
	serial_port = parameter.getParameter("serial_port", "/dev/ttyUSB0");
	serial_baudrate = parameter.getParameter("serial_baudrate", 115200);
	frame_id = parameter.getParameter("frame_id", "laser_frame");
	inverted = parameter.getParameter("inverted", false);
	angle_compensate = parameter.getParameter("angle_compensate", true);
}


bool SelidarApplication::checkSelidarHealth(SelidarDriver * drv)
{
    int op_result;
    SelidarHealth healthinfo;

    op_result = drv->getHealth(healthinfo);

    if (IS_OK(op_result)) {
        NS_NaviCommon::console.debug("Selidar health status : %d, errcode: %d \n",
        		healthinfo.status, healthinfo.err_code);

        if (healthinfo.status != StatusFine) {
            return false;
        } else {
            return true;
        }

    } else {
        return false;
    }
}

bool SelidarApplication::checkSelidarInfo(SelidarDriver * drv)
{
  int op_result;
  SelidarInfo device_info;

  op_result = drv->getDeviceInfo(device_info);

  if (IS_OK(op_result)) {
    NS_NaviCommon::console.debug("Selidar device info : \n");
    NS_NaviCommon::console.debug("\t model : %d \n", device_info.model);
    NS_NaviCommon::console.debug("\t hw ver : %d \n", device_info.hw_id);
    NS_NaviCommon::console.debug("\t fw ver : %d.%d \n", device_info.fw_major, device_info.fw_minor);
    return true;
  } else {
    return false;
  }

}


bool SelidarApplication::stopScanService(NS_ServiceType::RequestBase* request,
		NS_ServiceType::ResponseBase* response)
{
	if(!drv.isConnected())
       return false;

    drv.stop();

    return true;
}

bool SelidarApplication::startScanService(NS_ServiceType::RequestBase* request,
		NS_ServiceType::ResponseBase* response)
{
	if(!drv.isConnected())
       return false;

	NS_NaviCommon::console.message("Start motor");

    drv.startScan();
    return true;
}

void SelidarApplication::publishScan(SelidarMeasurementNode *nodes,
	                  size_t node_count, NS_NaviCommon::Time start,
	                  double scan_time, float angle_min, float angle_max)
{
	static int scan_count = 0;

	NS_DataType::LaserScan* scan_msg = new NS_DataType::LaserScan;

	scan_msg->header.stamp = start;
	scan_msg->header.frame_id = frame_id;
	scan_count++;

	bool reversed = (angle_max > angle_min);
	if ( reversed ) {
		scan_msg->angle_min =  M_PI - angle_max;
	    scan_msg->angle_max =  M_PI - angle_min;
	} else {
	    scan_msg->angle_min =  M_PI - angle_min;
	    scan_msg->angle_max =  M_PI - angle_max;
	}
	scan_msg->angle_increment =
			(scan_msg->angle_max - scan_msg->angle_min) / (double)(node_count - 1);
	//NS_NaviCommon::console.debug("publish_scan, scan_msg->angle_min: %f, scan_msg->angle_max: %f\n", scan_msg->angle_min, scan_msg->angle_max);
	scan_msg->scan_time = scan_time;
	scan_msg->time_increment = scan_time / (double)(node_count - 1);
	scan_msg->range_min = 0.15;
	scan_msg->range_max = 6.;

	scan_msg->intensities.resize(node_count);
	scan_msg->ranges.resize(node_count);
	bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
	if (!reverse_data) {
		for (size_t i = 0; i < node_count; i++) {
			float read_value = (float) nodes[i].distance_scale_1000/1000.0f;
			if (read_value == 0.0)
				scan_msg->ranges[i] = std::numeric_limits<float>::infinity();
			else
				scan_msg->ranges[i] = read_value;
		}

	} else {
		for (size_t i = 0; i < node_count; i++) {
			float read_value = (float)nodes[i].distance_scale_1000/1000.0f;
			if (read_value == 0.0)
				scan_msg->ranges[node_count-1-i] = std::numeric_limits<float>::infinity();
			else
				scan_msg->ranges[node_count-1-i] = read_value;
		}

	}

	dispitcher->publish(NS_NaviCommon::DATA_TYPE_LASER_SCAN, scan_msg);
}

void SelidarApplication::scanLoop()
{
	int op_result;

	drv.startScan();
	NS_NaviCommon::Time start_scan_time;
	NS_NaviCommon::Time end_scan_time;
	double scan_duration;
	while ( running ) {
		SelidarMeasurementNode nodes[360];
		size_t count = _countof(nodes);
		start_scan_time = NS_NaviCommon::Time::now();
		op_result = drv.grabScanData(nodes, count);
		end_scan_time = NS_NaviCommon::Time::now();
		scan_duration = (end_scan_time - start_scan_time).toSec() * 1e-3;
		if (op_result == Success) {
			float angle_min = DEG2RAD(0.0f);
			float angle_max = DEG2RAD(359.0f);

			angle_min = DEG2RAD((float)(nodes[0].angle_scale_100)/100.0f);
			angle_max = DEG2RAD((float)(nodes[count - 1].angle_scale_100)/100.0f);
			publishScan(nodes, count, start_scan_time, scan_duration, angle_min, angle_max);

		}
	}
}

void  SelidarApplication::initialize()
{
	NS_NaviCommon::console.message("selidar is initializing!");

	loadParameters();

	 // make connection...
	 if (IS_FAIL(drv.connect(serial_port.c_str(), (unsigned int)serial_baudrate, 0))) {
		 NS_NaviCommon::console.error("cannot bind to the specified serial port %s.", serial_port.c_str());
	 }

	 // check health...
	 /*
	 if (!checkSelidarHealth(&drv)) {
		return;
	 }
	 */

	 // get device info...
	 /*
	 if (!checkSelidarInfo(&drv)) {
		return;
	 }
	 */

	 service->advertise(NS_NaviCommon::SERVICE_TYPE_STOP_SCAN, boost::bind(&SelidarApplication::stopScanService, this, _1, _2));
	 service->advertise(NS_NaviCommon::SERVICE_TYPE_START_SCAN, boost::bind(&SelidarApplication::startScanService, this, _1, _2));

    initialized = true;

    NS_NaviCommon::console.message("selidar has initialized!");
}

void SelidarApplication::run()
{
	NS_NaviCommon::console.message("selidar is running!");

	running = true;

	scan_thread = boost::thread(boost::bind(&SelidarApplication::scanLoop, this));
}

void SelidarApplication::quit()
{
	NS_NaviCommon::console.message("selidar is quitting!");

    running = false;

    scan_thread.join();

    drv.stop();

    drv.disconnect();
}

}



