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
#include <Transform/LinearMath/Transform.h>
#include <DataSet/DataType/LaserScan.h>
#include <Time/Time.h>
#include <Time/Duration.h>

namespace NS_GMapping {

class GMappingApplication: public Application {
public:
  GMappingApplication();
  virtual ~GMappingApplication();
private:
  bool up_mounted;
  double max_range_;
  double max_u_range_;
  double maxrange_;
  double minimum_score_;
  double sigma_;
  int kernel_size_;
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
  double linear_update_;
  double angular_update_;
  double temporal_update_;
  double resample_threshold_;
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
  int throttle_scans_;
  NS_NaviCommon::Duration map_update_interval_;
  unsigned long int seed_;
private:
  void loadParameters();
  void laserDataCallback(NS_DataType::DataBase* laser_data);
  void odometryDataCallback(NS_DataType::DataBase* odometry_data);
  void mapService(NS_NaviCommon::RequestBase* request, NS_NaviCommon::ResponseBase* response);
  double computePoseEntropy();
  bool getOdomPose(OrientedPoint& gmap_pose);
  bool initMapper(NS_DataType::LaserScan& laser_data);
  bool addScan(NS_DataType::LaserScan& laser_data, OrientedPoint& gmap_pose);
  void updateMap(NS_DataType::LaserScan& laser_data);
private:
  GridSlamProcessor* gsp;
  RangeSensor* gsp_laser;
  OdometrySensor* gsp_odom;

  std::vector<double> laser_angles;
  bool do_reverse_range;
  unsigned int gsp_laser_beam_count;

  bool got_first_scan;
  bool got_map;

  NS_Transform::Transform map_to_odom;

  NS_Transform::Transform centered_laser_pose;

  int laser_count;

  boost::mutex map_to_odom_lock;
  boost::mutex map_lock;

public:
  virtual void initialize();
  virtual void run();
  virtual void quit();
};

} /* namespace NS_GMapping */

#endif /* MAPPING_GMAPPING_GMAPPINGAPPLICATION_H_ */
