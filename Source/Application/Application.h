/*
 * Application.h
 *
 *  Created on: 2016年9月29日
 *      Author: seeing
 */

#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include <Parameter/Parameter.h>
#include <DataSet/Dispitcher.h>
#include <Service/Service.h>

class Application
{
public:
	Application();
	virtual ~Application();

protected:
	bool running;
	bool initialized;

	NS_NaviCommon::Parameter parameter;
	NS_NaviCommon::Dispitcher* dispitcher;
	NS_NaviCommon::Service* service;

public:
	virtual void initialize();
	virtual void run();
	virtual void quit();

	bool isRunning()
	{
		return running;
	};

	bool isInitialized()
	{
		return initialized;
	};
};

#endif /* APPLICATION_APPLICATION_H_ */
