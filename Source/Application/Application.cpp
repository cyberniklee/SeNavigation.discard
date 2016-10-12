/*
 * Application.cpp
 *
 *  Created on: 2016年9月29日
 *      Author: seeing
 */

#include "Application.h"
#include <Console/Console.h>

static NS_NaviCommon::Dispitcher global_dispitcher;
static NS_NaviCommon::Service global_service;

Application::Application() {
	// TODO Auto-generated constructor stub
	dispitcher = &global_dispitcher;

}

Application::~Application() {
	// TODO Auto-generated destructor stub
}

void Application::initialize()
{
  NS_NaviCommon::console.warning("initializing with base application class!");
}

void Application::run()
{
  NS_NaviCommon::console.warning("running with base application class!");
}

void Application::quit()
{
  NS_NaviCommon::console.warning("quitting with base application class!");
}
