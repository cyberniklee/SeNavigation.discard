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
    NS_ServiceType::ResponseMap* respMap;
    unsigned char* mapStream;
    std::string mapDataFile;

  private:
    u_char*
    writeInPGM ();
    void
    saveMapInPGM ();
  };
}

#endif /* COMMUNICATORAPPLICATION_H_ */
