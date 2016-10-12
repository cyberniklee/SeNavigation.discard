/*
 * GMappingApplication.cpp
 *
 *  Created on: 2016年10月11日
 *      Author: seeing
 */

#include "GMappingApplication.h"
#include <Console/Console.h>

namespace NS_GMapping {

GMappingApplication::GMappingApplication()
{
	// TODO Auto-generated constructor stub

}

GMappingApplication::~GMappingApplication()
{
	// TODO Auto-generated destructor stub
}

void GMappingApplication::loadParameters()
{
  parameter.loadConfigurationFile("gmapping.xml");

}

void GMappingApplication::initialize()
{
  NS_NaviCommon::console.message("gmapping is initializing!");

  loadParameters();

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
