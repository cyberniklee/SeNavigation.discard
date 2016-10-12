#ifndef _SENSORLOG_H_
#define _SENSORLOG_H_

#include <list>
#include <istream>

#include "../Log/Configuration.h"
#include "../Sensor/SensorBase/SensorReading.h"
#include "../Sensor/SensorRange/RangeSensor.h"
#include "../Sensor/SensorOdometry/OdometrySensor.h"
#include "../Sensor/SensorOdometry/OdometryReading.h"
#include "../Sensor/SensorRange/RangeReading.h"

namespace NS_GMapping {

class SensorLog : public std::list<SensorReading*>{
	public:
		SensorLog(const SensorMap&);
		~SensorLog();
		std::istream& load(std::istream& is);
		OrientedPoint boundingBox(double& xmin, double& ymin, double& xmax, double& ymax) const;
	protected:
		const SensorMap& m_sensorMap;
		OdometryReading* parseOdometry(std::istream& is, const OdometrySensor* ) const;
		RangeReading* parseRange(std::istream& is, const RangeSensor* ) const;
};

};

#endif
