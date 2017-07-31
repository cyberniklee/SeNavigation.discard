/*
 * CommunicatiorApplication.cpp
 *
 *  Created on: 2016年10月31日
 *      Author: nico
 */

#include <Console/Console.h>
#include <Service/Service.h>
#include <vector>
#include <stdlib.h>
#include <DataSet/DataType/PoseStamped.h>
#include <Transform/DataTypes.h>
#include "TestApplication.h"

namespace NS_Test
{
  
  TestApplication::TestApplication ()
  {

  }
  
  TestApplication::~TestApplication ()
  {
    // TODO
  }
  
  void
  TestApplication::loadParameters ()
  {
    parameter.loadConfigurationFile ("test.xml");
    map_file_ = parameter.getParameter ("map_file", "/tmp/gmap.pgm");
    map_gen_freq_ = parameter.getParameter ("map_gen_freq", 1.0f);
  }
  
  void
  TestApplication::saveMapInPGM (NS_DataType::OccupancyGrid& map,
                                         std::string map_file)
  {
    FILE* out = fopen (map_file.c_str (), "w");
    if (!out)
    {
      NS_NaviCommon::console.warning ("Couldn't save map file to %s",
                                      map_file.c_str ());
    }
    fprintf (out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
             map.info.resolution, map.info.width, map.info.height);
    
    for (unsigned long y = 0; y < map.info.height; y++)
    {
      for (unsigned long x = 0; x < map.info.width; x++)
      {
        unsigned long i = x + (map.info.height - y - 1) * map.info.width;
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
  TestApplication::mapGenerateLoop (double frequency)
  {
    NS_NaviCommon::Rate rate (frequency);
    int file_no = 0;
    while (running)
    {
      NS_ServiceType::ResponseMap map_resp;

      service->call (SERVICE_TYPE_MAP, NULL, &map_resp);

      if (map_resp.result)
      {
        saveMapInPGM (map_resp.map, map_file_);
        file_no++;
        NS_NaviCommon::console.message ("Save map file No: %d!", file_no);
      }
      rate.sleep ();
    }
  }

  void
  TestApplication::initialize ()
  {
    NS_NaviCommon::console.message ("Test application is initializing!");
    
    loadParameters ();
    
    initialized = true;
  }
  
  void
  TestApplication::run ()
  {
    NS_NaviCommon::console.message ("Test application is running!");
    Application::running = true;
    
    map_gen_thread = boost::thread (
        boost::bind (&TestApplication::mapGenerateLoop, this, map_gen_freq_));
  }
  
  void
  TestApplication::quit ()
  {
    NS_NaviCommon::console.message ("Test application is quitting!");
    
    Application::running = false;

    map_gen_thread.join ();
  }
}

