/*
 * Application.cpp
 *
 *  Created on: 2016年9月29日
 *      Author: seeing
 */

#include "Application.h"
#include <Console/Console.h>
#include <Time/Time.h>

static NS_NaviCommon::Dispitcher global_dispitcher;
static NS_NaviCommon::Service global_service;

Application::Application()
{
  dispitcher = &global_dispitcher;
  service = &global_service;
}

Application::~Application()
{
  dispitcher = NULL;
  service = NULL;
}

void Application::globalInitialize()
{
  NS_NaviCommon::Time::init();

  global_dispitcher.initialize();
  global_service.initialize();
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
