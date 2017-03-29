#ifndef _CONFIGURATION_H_
#define _CONFIGURATION_H_

#include <istream>

#include "../Sensor/SensorBase/Sensor.h"

namespace NS_GMapping
{
  
  class Configuration
  {
  public:
    virtual
    ~Configuration ();
    virtual SensorMap
    computeSensorMap () const=0;
  };

}
;
#endif

