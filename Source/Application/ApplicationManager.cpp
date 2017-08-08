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

void
ApplicationManager::signalAction (int signal)
{
  NS_NaviCommon::console.warning ("received a signal, id:%d!", signal);
  instance->quitApplications ();
}

void
ApplicationManager::applicationsPending ()
{
  Rate rate (10);
  while (running)
  {
    rate.sleep ();
  }
}

ApplicationManager::ApplicationManager ()
{
  // TODO Auto-generated constructor stub
  instance = this;
  running = false;
}

ApplicationManager::~ApplicationManager ()
{
  // TODO Auto-generated destructor stub
}

void
ApplicationManager::registerSignal ()
{
  signal (SIGINT, signalAction);
}

void
ApplicationManager::quitApplications ()
{
  std::vector<Application*>::iterator it;
  for (it = applications.begin (); it != applications.end (); it++)
  {
    (*it)->quit ();
  }
  running = false;
  applications_pending_thread.join ();
}

bool
ApplicationManager::initialize ()
{
  Application::globalInitialize ();
  registerSignal ();
  registerApplications ();
  
  std::vector<Application*>::iterator it;
  for (it = applications.begin (); it != applications.end (); it++)
  {
    (*it)->initialize ();
    if ((*it)->isInitialized () == false)
    {
      NS_NaviCommon::console.error ("Initialize not complete!");
      return false;
    }
  }
  return true;
}

bool
ApplicationManager::run ()
{
  std::vector<Application*>::iterator it;
  for (it = applications.begin (); it != applications.end (); it++)
  {
    (*it)->run ();
    if ((*it)->isRunning () == false)
    {
      NS_NaviCommon::console.error ("Application not set running!");
      return false;
    }
  }
  
  running = true;
  
  applications_pending_thread = boost::thread (
      boost::bind (&ApplicationManager::applicationsPending, this));
  
  return true;
}

void
ApplicationManager::pending ()
{
  applications_pending_thread.join ();
}
