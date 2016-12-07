/*
 * NavigationApplication.h
 *
 *  Created on: 2016年12月3日
 *      Author: seeing
 */

#ifndef _NAVIGATIONAPPLICATION_H_
#define _NAVIGATIONAPPLICATION_H_

#include "../Application/Application.h"

namespace NS_Navigation {

class NavigationApplication: public Application {
public:
  NavigationApplication();
  virtual ~NavigationApplication();
private:
  void loadParameters();
public:
  virtual void initialize();
  virtual void run();
  virtual void quit();
};

} /* namespace NS_Navigation */

#endif /* NAVIGATION_NAVIGATIONAPPLICATION_H_ */
