#ifndef _CARMENCONFIGURATION_H_
#define _CARMENCONFIGURATION_H_

#include <string>
#include <map>
#include <vector>
#include <istream>

#include "../Sensor/SensorBase/Sensor.h"
#include "Configuration.h"

namespace NS_GMapping
{
  
  class CarmenConfiguration: public Configuration, public std::map<std::string,
      std::vector<std::string> >
  {
  public:
    virtual std::istream&
    load (std::istream& is);
    virtual SensorMap
    computeSensorMap () const;
  };

}
;

#endif

