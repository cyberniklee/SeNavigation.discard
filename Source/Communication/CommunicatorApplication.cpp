/*
 * CommunicatiorApplication.cpp
 *
 *  Created on: 2016年10月31日
 *      Author: nico
 */

#include "CommunicatorApplication.h"
#include <Console/Console.h>
#include <Service/Service.h>
#include <vector>
#include <stdlib.h>

namespace NS_Communication
{
  
  CommunicatorApplication::CommunicatorApplication ()
  {
    Communicator::instance = this;
  }
  
  CommunicatorApplication::~CommunicatorApplication ()
  {
    // TODO
  }
  
  void
  CommunicatorApplication::loadParameters ()
  {
    local_port_ = parameter.getParameter ("local_port", 6689);
    remote_port_ = parameter.getParameter ("remote_port", 6688);
    map_file_ = parameter.getParameter ("map_file", "/tmp/gmap.pgm");
  }
  
  void
  CommunicatorApplication::onTimeout (CommData* timeout_message)
  {
    if (timeout_message != NULL)
      delete timeout_message;
  }
  
  void
  CommunicatorApplication::saveMapInPGM (NS_DataType::OccupancyGrid& map, std::string map_file)
  {
    FILE* out = fopen (map_file.c_str (), "w");
    if (!out)
    {
      NS_NaviCommon::console.warning ("Couldn't save map file to %s",
                                      map_file.c_str ());
    }
    fprintf (out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
             map.info.resolution, map.info.width,
             map.info.height);

    for (unsigned long y = 0; y < map.info.height; y++)
    {
      for (unsigned long x = 0; x < map.info.width; x++)
      {
        unsigned long i = x
            + (map.info.height - y - 1) * map.info.width;
        if (map.data[i] == 0)
        { //occ [0,0.1)
          fputc (254, out);
        }
        else if (map.data[i] == +100)
        { //occ (0.65,1]
          fputc (000, out);
        }
        else
        { //occ [0.1,0.65]
          fputc (205, out);
        }
      }
    }
    
    fclose (out);
    NS_NaviCommon::console.debug ("write PGM finish, path: %s",
                                    map_file_.c_str ());
    
  }
  
  void
  CommunicatorApplication::onReceive (CommData* message)
  {
    if (message != NULL)
    {
      if (message->reason == COMMUNICATION_DATA_REASON_MAP_SIZE)
      {
        NS_ServiceType::ResponseMap map_resp;
        char mapSize_str[16] = {0};
        CommData* response = this->createResponseByRequest (message);
        
        if (service->call (SERVICE_TYPE_MAP, NULL, &map_resp))
        {
          response->payload_length = sizeof(map_resp.map.data.size ());
          saveMapInPGM(map_resp.map, map_file_);
          sprintf (mapSize_str, "%ld", map_resp.map.data.size ());
          memcpy (response->payload, mapSize_str, sizeof(mapSize_str));
        }else{
          response->payload_length = 0;
        }

        this->sendResponse (response);
      }
      else if (message->reason == COMMUNICATION_DATA_REASON_MAP)
      {
        CommData* response = this->createResponseByRequest (message);

        const char* map_path = (const char*) map_file_.c_str ();
        unsigned int len = map_file_.length ();
        memcpy (response->payload, map_path, len);

        response->payload_length = len;

        this->sendResponse (response);
      }
      else if (message->reason == COMMUNICATION_DATA_REASON_MAP_META)
      {
        NS_NaviCommon::console.message (
            "receive message: COMMUNICATION_DATA_REASON_MAP_META!");
      }else{
        NS_NaviCommon::console.error (
            "invalid message: reason %d !", message->reason);
      }
    }
  }
  
  void
  CommunicatorApplication::initialize ()
  {
    NS_NaviCommon::console.message ("Communion is initializing!");
    
    loadParameters ();
    Communicator::initialize (local_port_, remote_port_);

    initialized = true;
    
  }
  
  void
  CommunicatorApplication::run ()
  {
    NS_NaviCommon::console.message ("Communion is running!");
    Application::running = true;
    
  }
  
  void
  CommunicatorApplication::quit ()
  {
    NS_NaviCommon::console.message ("Communion is quitting!");
    
    Application::running = false;
  }
}

