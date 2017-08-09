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
#include <MapGenerator/MapGenerator.h>

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
        NS_NaviCommon::MapGenerator::saveMapInPGM (map_resp.map.data, map_resp.map.info.height, map_resp.map.info.width, map_file_);
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

