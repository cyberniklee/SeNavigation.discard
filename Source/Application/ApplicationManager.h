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
#include "../Mapping/GMapping/GMappingApplication.h"
#include "../Controller/ControllerApplication.h"

using namespace NS_GMapping;
using namespace NS_Controller;

class ApplicationManager
{
public:
  ApplicationManager();
  virtual ~ApplicationManager();

private:
  std::vector<Application*> applications;

  void registerApplications()
  {
	GMappingApplication* gmapping = new GMappingApplication;
    applications.push_back(gmapping);

    ControllerApplication* controller = new ControllerApplication;
    applications.push_back(controller);
  }

  void quitApplications();

  static void signalAction(int signal);

  void registerSignal();

public:

  void initialize();
  void run();
};

#endif /* APPLICATION_APPLICATIONMANAGER_H_ */
