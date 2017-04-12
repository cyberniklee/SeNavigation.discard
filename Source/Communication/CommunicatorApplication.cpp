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
  
  u_char*
  CommunicatorApplication::writeInPGM ()
  {
    unsigned long mapSize = respMap->map.data.size ();
    unsigned char* out = new unsigned char[mapSize];
    
    unsigned long count = 0;
    out[count++] = respMap->map.info.resolution;
    out[count++] = respMap->map.info.width;
    out[count++] = respMap->map.info.height;
    
    for (unsigned long y = 0; y < respMap->map.info.height; y++)
    {
      for (unsigned long x = 0; x < respMap->map.info.width; x++)
      {
        unsigned long i = x
            + (respMap->map.info.height - y - 1) * respMap->map.info.width;
        if (respMap->map.data[i] == 0)
        { //occ [0,0.1)
          out[count++] = 254;
        }
        else if (respMap->map.data[i] == +100)
        { //occ (0.65,1]
          out[count++] = 000;
        }
        else
        { //occ [0.1,0.65]
          out[count++] = 205;
        }
      }
    }
    
    return out;
  }
  
  void
  CommunicatorApplication::saveMapInPGM ()
  {
    FILE* out = fopen (map_file_.c_str (), "w");
    if (!out)
    {
      NS_NaviCommon::console.warning ("Couldn't save map file to %s",
                                      map_file_.c_str ());
    }
    fprintf (out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
             respMap->map.info.resolution, respMap->map.info.width,
             respMap->map.info.height);

    for (unsigned long y = 0; y < respMap->map.info.height; y++)
    {
      for (unsigned long x = 0; x < respMap->map.info.width; x++)
      {
        unsigned long i = x
            + (respMap->map.info.height - y - 1) * respMap->map.info.width;
        if (respMap->map.data[i] == 0)
        { //occ [0,0.1)
          fputc (254, out);
        }
        else if (respMap->map.data[i] == +100)
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
    NS_NaviCommon::console.message ("write PGM finish, path: %s",
                                    map_file_.c_str ());
    
  }
  
  void
  CommunicatorApplication::onReceive (CommData* message)
  {
    if (message != NULL)
    {
      if (message->reason == COMMUNICATION_DATA_REASON_MAP_SIZE)
      {
        if (respMap == NULL)
          respMap = new NS_ServiceType::ResponseMap;
        
        service->call (SERVICE_TYPE_MAP, NULL, respMap);
        CommData* response = this->createResponseByRequest (message);
        response->payload_length = sizeof(respMap->map.data.size ());
        
        mapStream = writeInPGM (); // transform respMap to map stream of PGM format.

        char* mapSize_str = new char[10];
        sprintf (mapSize_str, "%ld", respMap->map.data.size ());
        memcpy (response->payload, mapSize_str, 10);
        this->sendResponse (response);
        
      }
      else if (message->reason == COMMUNICATION_DATA_REASON_MAP)
      {
        if (respMap == NULL)
          NS_NaviCommon::console.message (
              "you have to get map size before request Map data!");
        
        CommData* response = this->createResponseByRequest (message);
        
        saveMapInPGM ();
        const u_char* mapPath = (const u_char*) map_file_.c_str ();
        unsigned int len = map_file_.length ();
        memcpy (response->payload, mapPath, len);
        
        NS_NaviCommon::console.debug ("receive mapPath len: %d", len);
        NS_NaviCommon::console.debug ("receive mapPath: %s", mapPath);
        NS_NaviCommon::console.debug ("receive payload: %s",
                                        response->payload);
        response->payload_length = len;
        
        //process for PGM stream
        /*
         unsigned long dataStart = atoi((char*)message->payload);
         NS_NaviCommon::console.message("Map data start from : %ld!", dataStart);//--
         unsigned long mapSize = respMap->map.data.size();
         unsigned long dataEnd = (mapSize-dataStart > 500)? dataStart + 500 : mapSize;
         //NS_NaviCommon::console.message("mapSize is : %ld, dataEnd is %ld!", mapSize,dataEnd);//--

         CommData* response = this->createResponseByRequest(message);
         response->payload_length = dataEnd - dataStart;
         //NS_NaviCommon::console.message("dataSize is : %ld", response->payload_length);//--
         for (unsigned long i = dataStart, j = 0; j < response->payload_length; ++i, ++j)
         response->payload[j] = mapStream[i];
         */

        this->sendResponse (response);
        
      }
      else if (message->reason == COMMUNICATION_DATA_REASON_MAP_META)
      {
        NS_NaviCommon::console.message (
            "receive message: COMMUNICATION_DATA_REASON_MAP_META!");
      }
      
    }
    
  }
  
  void
  CommunicatorApplication::initialize ()
  {
    NS_NaviCommon::console.message ("Communion is initializing!");
    respMap = NULL;
    mapStream = NULL;
    
    loadParameters ();
    instance->initialize (local_port_, remote_port_);

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

