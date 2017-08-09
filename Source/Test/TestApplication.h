/*
 * TestApplication.h
 *
 *  Created on: 2016年10月31日
 *      Author: nico
 */

#ifndef _TEST_APPLICATION_H_
#define _TEST_APPLICATION_H_

#include "../Application/Application.h"
#include <boost/thread/thread.hpp>
#include <Communication/Communicator.h>
#include <Communication/DataType/DataTypes.h>
#include <Communication/NetTranceiver.h>
#include <Service/ServiceType/ResponseMap.h>
#include <DataSet/DataType/OccupancyGrid.h>

namespace NS_Test
{
  using namespace NS_CommDataType;
  using namespace NS_NaviCommon;
  
  class TestApplication: public Application
  {
    
  public:
    TestApplication ();
    ~TestApplication ();
    void
    loadParameters ();

  public:
    virtual void
    initialize ();
    virtual void
    run ();
    virtual void
    quit ();

  private:
    std::string map_file_;
    double map_gen_freq_;

    boost::thread map_gen_thread;

  private:
    void
    mapGenerateLoop (double frequency);
  };
}

#endif /* COMMUNICATORAPPLICATION_H_ */
