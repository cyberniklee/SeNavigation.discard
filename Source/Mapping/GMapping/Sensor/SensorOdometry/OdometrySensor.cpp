#include "../SensorOdometry/OdometrySensor.h"

namespace NS_GMapping
{
  
  OdometrySensor::OdometrySensor (const std::string& name, bool ideal)
      : Sensor (name)
  {
    m_ideal = ideal;
  }

}
;

