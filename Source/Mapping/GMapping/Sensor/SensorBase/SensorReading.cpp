#include "../../Sensor/SensorBase/SensorReading.h"

namespace NS_GMapping{

SensorReading::SensorReading(const Sensor* s, double t){
	m_sensor=s;
	m_time=t;
}


SensorReading::~SensorReading(){
}

};

