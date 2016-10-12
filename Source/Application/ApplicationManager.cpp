/*
 * ApplicationManager.cpp
 *
 *  Created on: 2016年9月24日
 *      Author: seeing
 */

#include "ApplicationManager.h"
#include <Console/Console.h>
#include "stdio.h"
#include "signal.h"

static ApplicationManager* instance;

void ApplicationManager::signalAction(int signal)
{
  NS_NaviCommon::console.warning("received a signal, id:%d!", signal);
  instance->quitApplications();
}

ApplicationManager::ApplicationManager() {
	// TODO Auto-generated constructor stub
  instance = this;
}

ApplicationManager::~ApplicationManager() {
	// TODO Auto-generated destructor stub
}

void ApplicationManager::registerSignal()
{
  signal(SIGINT, signalAction);
}

void ApplicationManager::quitApplications()
{
  std::vector<Application*>::iterator it;
  for(it = applications.begin(); it != applications.end(); it++)
  {
	(*it)->quit();
  }
}

void ApplicationManager::initialize()
{
  Application::globalInitialize();
  registerSignal();
  registerApplications();

  std::vector<Application*>::iterator it;
  for(it = applications.begin(); it != applications.end(); it++)
  {
	(*it)->initialize();
  }
}

void ApplicationManager::run()
{
  std::vector<Application*>::iterator it;
  for(it = applications.begin(); it != applications.end(); it++)
  {
    (*it)->run();
  }
}
