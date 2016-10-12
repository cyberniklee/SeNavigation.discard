/*
 * GMappingApplication.h
 *
 *  Created on: 2016年10月11日
 *      Author: seeing
 */

#ifndef _GMAPPINGAPPLICATION_H_
#define _GMAPPINGAPPLICATION_H_

#include "../../Application/Application.h"

#include "GridFastSlam/GridSlamProcessor.h"

#include <DataSet/DataType/DataBase.h>
#include <Service/ServiceType/RequestBase.h>
#include <Service/ServiceType/ResponseBase.h>

namespace NS_GMapping {

class GMappingApplication: public Application {
public:
  GMappingApplication();
  virtual ~GMappingApplication();
private:
  double maxRange_;
  double maxUrange_;
  double maxrange_;
  double minimum_score_;
  double sigma_;
  int kernelSize_;
  double lstep_;
  double astep_;
  int iterations_;
  double lsigma_;
  double ogain_;
  int lskip_;
  double srr_;
  double srt_;
  double str_;
  double stt_;
  double linearUpdate_;
  double angularUpdate_;
  double temporalUpdate_;
  double resampleThreshold_;
  int particles_;
  double xmin_;
  double ymin_;
  double xmax_;
  double ymax_;
  double delta_;
  double occ_thresh_;
  double llsamplerange_;
  double llsamplestep_;
  double lasamplerange_;
  double lasamplestep_;
private:
  void loadParameters();
  void laserDataCallback(NS_NaviCommon::DataBase* laser_data);
  void mapService(NS_NaviCommon::RequestBase* request, NS_NaviCommon::ResponseBase* response);
private:
  NS_GMapping::GridSlamProcessor* gsp;
  NS_GMapping::RangeSensor* gsp_laser;
  NS_GMapping::OdometrySensor* gsp_odom;
public:
  virtual void initialize();
  virtual void run();
  virtual void quit();
};

} /* namespace NS_GMapping */

#endif /* MAPPING_GMAPPING_GMAPPINGAPPLICATION_H_ */
