/*
 * HectorMappingApplication.h
 *
 *  Created on: 2016年10月11日
 *      Author: seeing
 */

#ifndef _HECTORMAPPINGAPPLICATION_H_
#define _HECTORMAPPINGAPPLICATION_H_

#include "../../Application/Application.h"

#include <DataSet/DataType/DataBase.h>
#include <Service/ServiceType/RequestBase.h>
#include <Service/ServiceType/ResponseBase.h>
#include <Transform/LinearMath/Transform.h>
#include <DataSet/DataType/LaserScan.h>
#include <DataSet/DataType/OccupancyGrid.h>
#include <Time/Time.h>
#include <Time/Duration.h>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/thread/mutex.hpp>

#include "Scan/DataPointContainer.h"
#include "Utils/HectorDebugInfoInterface.h"
#include "Utils/HectorDrawings.h"
#include "Slam/HectorSlamProcessor.h"
#include "Utils/PoseInfoContainer.h"
#include "Utils/HectorMapMutex.h"

#include <DataSet/DataType/Point32.h>

namespace NS_HectorMapping
{
  
  class HectorMappingApplication: public Application
  {
  public:
    HectorMappingApplication ();
    virtual
    ~HectorMappingApplication ();
  private:
    HectorDebugInfoInterface* debugInfoProvider;

    HectorDrawings* hectorDrawings;

    int lastGetMapUpdateIndex;

    PoseInfoContainer poseInfoContainer_;

    HectorSlamProcessor* slamProcessor;
    DataContainer laserScanContainer;

    Eigen::Vector3f lastSlamPose;

    void
    getMapInfo (NS_DataType::OccupancyGrid& map,
                const NS_HectorMapping::GridMap& gridMap);

    bool
    pointsToDataContainer (const std::vector<NS_DataType::Point32>& points,
                           const NS_Transform::StampedTransform& laserTransform,
                           DataContainer& dataContainer, float scaleToMap);

    void
    projectLaser (const NS_DataType::LaserScan& scan_in,
                  std::vector<NS_DataType::Point32>& points,
                  double range_cutoff, bool preservative);

    const boost::numeric::ublas::matrix<double>&
    getUnitVectors (double angle_min, double angle_max, double angle_increment,
                    unsigned int length);

    bool
    laserScanToDataContainer (const NS_DataType::LaserScan& scan,
                              DataContainer& dataContainer, float scaleToMap);

    void
    updateMap (NS_DataType::OccupancyGrid& map,
               const NS_HectorMapping::GridMap& gridMap,
               MapLockerInterface* mapMutex);

  private:
    //Parameters related to publishing the scanmatcher pose directly via tf
    
    double update_factor_free_;
    double update_factor_occupied_;
    double map_update_distance_threshold_;
    double map_update_angle_threshold_;

    double map_resolution_;
    int map_size_;
    double map_start_x_;
    double map_start_y_;
    int map_multi_res_levels_;

    double map_pub_period_;

    bool use_tf_scan_transformation_;
    bool use_tf_pose_start_estimate_;
    bool map_with_known_poses_;
    bool timing_output_;

    float sqr_laser_min_dist_;
    float sqr_laser_max_dist_;
    float laser_z_min_value_;
    float laser_z_max_value_;

    double map_update_frequency_;

  private:
    std::map<std::string, boost::numeric::ublas::matrix<double>*> unit_vector_map;
    boost::mutex guv_mutex;
  private:
    void
    loadParameters ();
    void
    laserDataCallback (NS_DataType::DataBase* laser_data);

    void
    mapService (NS_ServiceType::RequestBase* request,
                NS_ServiceType::ResponseBase* response);

    void
    mapTransformService (NS_ServiceType::RequestBase* request,
                         NS_ServiceType::ResponseBase* response);

    void
    updateMapLoop (double frequency);

  private:
    NS_Transform::Transform map_to_odom;

    NS_Transform::Transform centered_laser_pose;

    int laser_count;

    NS_DataType::OccupancyGrid map;

    boost::mutex map_to_odom_lock;
    boost::mutex map_lock;

    boost::thread update_map_thread;

  public:
    virtual void
    initialize ();
    virtual void
    run ();
    virtual void
    quit ();
  };

}

#endif
