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
#include "../Controller/ControllerApplication.h"

using namespace NS_GMapping;
using namespace NS_Controller;
using namespace NS_Selidar;

class ApplicationManager
{
public:
  ApplicationManager();
  virtual ~ApplicationManager();

private:
  bool running;

  std::vector<Application*> applications;

  boost::thread applications_pending_thread;

  void applicationsPending();

  void registerApplications()
  {
    SelidarApplication* selidar = new SelidarApplication;
    applications.push_back(selidar);

	GMappingApplication* gmapping = new GMappingApplication;
    applications.push_back(gmapping);

    ControllerApplication* controller = new ControllerApplication;
    applications.push_back(controller);
  }

  void quitApplications();

  static void signalAction(int signal);

  void registerSignal();

public:

  bool initialize();
  bool run();

  void pending();
};

#endif /* APPLICATION_APPLICATIONMANAGER_H_ */
