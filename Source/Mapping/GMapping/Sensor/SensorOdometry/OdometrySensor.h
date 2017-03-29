#ifndef _ODOMETRYSENSOR_H_
#define _ODOMETRYSENSOR_H_

#include <string>

#include "../../Sensor/SensorBase/Sensor.h"

namespace NS_GMapping
{
  
  class OdometrySensor: public Sensor
  {
  public:
    OdometrySensor (const std::string& name, bool ideal = false);
    inline bool
    isIdeal () const
    {
      return m_ideal;
    }
  protected:
    bool m_ideal;
  };

}
;

#endif

