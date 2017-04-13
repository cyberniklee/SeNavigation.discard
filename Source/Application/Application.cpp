/*
 * Application.cpp
 *
 *  Created on: 2016年9月29日
 *      Author: seeing
 */

#include "Application.h"
#include <Console/Console.h>
#include <Time/Time.h>
#include <Parameter/Parameter.h>

static NS_NaviCommon::Dispitcher global_dispitcher;
static NS_NaviCommon::Service global_service;

Application::Application ()
{
  dispitcher = &global_dispitcher;
  service = &global_service;
}

Application::~Application ()
{
  dispitcher = NULL;
  service = NULL;
}

void
Application::globalInitialize ()
{
  NS_NaviCommon::Parameter para;
  para.loadConfigurationFile ("application.xml");

  if (para.getParameter ("disable_stdout", 1) == 1)
  {
    NS_NaviCommon::disableStdoutStream ();
    NS_NaviCommon::console.warning ("Stdout stream has been disabled!");
  }

  string redir_log = para.getParameter ("redirect_stdout_file", "");
  if (redir_log != "")
  {
    NS_NaviCommon::redirectStdoutStream (redir_log);
    NS_NaviCommon::console.warning ("Stdout stream has been redirected to %s!", redir_log.c_str());
  }

  NS_NaviCommon::Time::init ();
  
  global_dispitcher.initialize ();
  global_service.initialize ();
}

void
Application::initialize ()
{
  NS_NaviCommon::console.warning ("initializing with base application class!");
}

void
Application::run ()
{
  NS_NaviCommon::console.warning ("running with base application class!");
}

void
Application::quit ()
{
  NS_NaviCommon::console.warning ("quitting with base application class!");
}
