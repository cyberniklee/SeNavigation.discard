/*
 * GMappingApplication.cpp
 *
 *  Created on: 2016年10月11日
 *      Author: seeing
 */

#include "GMappingApplication.h"
#include <Console/Console.h>
#include <boost/bind.hpp>

namespace NS_GMapping {

GMappingApplication::GMappingApplication()
{
  gsp = new NS_GMapping::GridSlamProcessor();

  gsp_laser = NULL;
  gsp_odom = NULL;

  got_first_scan = false;
  got_map = false;
}

GMappingApplication::~GMappingApplication()
{
  if(gsp)
    delete gsp;
  if(gsp_laser)
    delete gsp_laser;
  if(gsp_odom)
    delete gsp_odom;

}

void GMappingApplication::laserDataCallback(NS_NaviCommon::DataBase* laser_data)
{

}

void GMappingApplication::odometryDataCallback(NS_NaviCommon::DataBase* odometry_data)
{

}

void GMappingApplication::mapService(NS_NaviCommon::RequestBase* request, NS_NaviCommon::ResponseBase* response)
{

}

void GMappingApplication::loadParameters()
{
  parameter.loadConfigurationFile("GMapping.xml");

}

void GMappingApplication::initialize()
{
  NS_NaviCommon::console.message("gmapping is initializing!");

  loadParameters();

  dispitcher->subscribe(NS_NaviCommon::DATA_TYPE_LASER_SCAN,
		  boost::bind(&GMappingApplication::laserDataCallback, this, _1));

  dispitcher->subscribe(NS_NaviCommon::DATA_TYPE_ODOMETRY,
  		  boost::bind(&GMappingApplication::odometryDataCallback, this, _1));

  service->advertise(NS_NaviCommon::SERVICE_TYPE_MAP,
		  boost::bind(&GMappingApplication::mapService, this, _1, _2));

  initialized = true;
}

void GMappingApplication::run()
{
  NS_NaviCommon::console.message("gmapping is running!");
  running = true;
  while(running);
}

void GMappingApplication::quit()
{
  NS_NaviCommon::console.message("gmapping is quitting!");
  running = false;
}

} /* namespace NS_GMapping */
