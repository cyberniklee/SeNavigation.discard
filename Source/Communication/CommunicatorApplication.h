/*
 * CommunicatorApplication.h
 *
 *  Created on: 2016年10月31日
 *      Author: nico
 */

#ifndef COMMUNICATORAPPLICATION_H_
#define COMMUNICATORAPPLICATION_H_

#include "../Application/Application.h"
#include <boost/thread/thread.hpp>
#include <Communication/Communicator.h>
#include <Communication/DataType/DataTypes.h>
#include <Communication/NetTranceiver.h>
#include <Service/ServiceType/ResponseMap.h>
#include <DataSet/DataType/OccupancyGrid.h>

namespace NS_Communication
{
  using namespace NS_CommDataType;
  using namespace NS_NaviCommon;
  
  class CommunicatorApplication: public Application,
      public NS_NaviCommon::Communicator
  {
    
  public:
    CommunicatorApplication ();
    ~CommunicatorApplication ();
    void
    loadParameters ();
    virtual void
    onTimeout (CommData* timeout_message);
    virtual void
    onReceive (CommData* message);

  public:
    virtual void
    initialize ();
    virtual void
    run ();
    virtual void
    quit ();

  private:
    int local_port_;
    int remote_port_;
    std::string map_file_;

  private:
    void
    saveMapInPGM (NS_DataType::OccupancyGrid& map, std::string map_file);
  };
}

#endif /* COMMUNICATORAPPLICATION_H_ */
