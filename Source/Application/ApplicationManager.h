/*
 * ApplicationManager.h
 *
 *  Created on: 2016年9月24日
 *      Author: seeing
 */

#ifndef _APPLICATIONMANAGER_H_
#define _APPLICATIONMANAGER_H_

#include "Application.h"
#include <vector>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include "../Sensor/Lidar/SelidarApplication.h"
#include "../Mapping/GMapping/GMappingApplication.h"
#include "../Mapping/Hector/HectorMappingApplication.h"
#include "../Controller/ControllerApplication.h"
#include "../Navigation/NavigationApplication.h"
#include "../Communication/CommunicatorApplication.h"
#include "../Test/TestApplication.h"

using namespace NS_GMapping;
using namespace NS_Controller;
using namespace NS_Selidar;
using namespace NS_Navigation;
using namespace NS_Communication;
using namespace NS_Test;
using namespace NS_HectorMapping;

class ApplicationManager
{
public:
  ApplicationManager ();
  virtual
  ~ApplicationManager ();

private:
  bool running;

  std::vector<Application*> applications;

  boost::thread applications_pending_thread;

  void
  applicationsPending ();

  void
  registerApplications ()
  {
    CommunicatorApplication* communicator = new CommunicatorApplication;
    applications.push_back (communicator);

    SelidarApplication* selidar = new SelidarApplication;
    applications.push_back (selidar);

    GMappingApplication* gmapping = new GMappingApplication;
    applications.push_back (gmapping);
/*
    HectorMappingApplication* hector = new HectorMappingApplication;
    applications.push_back (hector);
*/
    ControllerApplication* controller = new ControllerApplication;
    applications.push_back (controller);

    NavigationApplication* navigation = new NavigationApplication;
    applications.push_back (navigation);

    TestApplication* test = new TestApplication;
    applications.push_back (test);
  }
  
  void
  quitApplications ();

  static void
  signalAction (int signal);

  void
  registerSignal ();

public:
  
  bool
  initialize ();
  bool
  run ();

  void
  pending ();
};

#endif /* APPLICATION_APPLICATIONMANAGER_H_ */
