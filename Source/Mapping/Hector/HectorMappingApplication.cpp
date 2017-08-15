#include "HectorMappingApplication.h"
#include <Console/Console.h>
#include <boost/bind.hpp>
#include <Transform/DataTypes.h>
#include <Transform/LinearMath/Transform.h>
#include <DataSet/DataType/Odometry.h>
#include <Service/ServiceType/RequestMap.h>
#include <Service/ServiceType/ResponseMap.h>
#include <Service/ServiceType/RequestTransform.h>
#include <Service/ServiceType/ResponseTransform.h>
#include <Service/Service.h>
#include <time.h>

namespace NS_HectorMapping
{
  
  HectorMappingApplication::HectorMappingApplication ()
  {
    hectorDrawings = NULL;
    debugInfoProvider = NULL;
    lastGetMapUpdateIndex = -100;
  }
  
  HectorMappingApplication::~HectorMappingApplication ()
  {
    delete slamProcessor;
    
    if (hectorDrawings)
      delete hectorDrawings;
    
    if (debugInfoProvider)
      delete debugInfoProvider;
  }
  
  void
  HectorMappingApplication::laserDataCallback (
      NS_DataType::DataBase* laser_data)
  {
    NS_DataType::LaserScan* laser = (NS_DataType::LaserScan*) laser_data;
    laser_count++;
///////////////////////////////////////////////////////////////////////////
    /*
     for(int i = 0; i < laser->ranges.size(); i++)
     {
     float degree = (laser->angle_min + laser->angle_increment * i);
     NS_NaviCommon::console.debug("--->   angle: %f, range: %f", degree, laser->ranges[i]);
     }
     return;
     */
////////////////////////////////////////////////////////////////////////////
    
    NS_Transform::StampedTransform laserTransform;

    laserTransform.setIdentity ();

    std::vector<NS_DataType::Point32> laser_points;
    
    projectLaser (*laser, laser_points, 6.0f, false);
    
    Eigen::Vector3f startEstimate (Eigen::Vector3f::Zero ());
    
    if (pointsToDataContainer (laser_points, laserTransform, laserScanContainer,
                               slamProcessor->getScaleToMap ()))
    {
      startEstimate = slamProcessor->getLastScanMatchPose ();
      
      slamProcessor->update (laserScanContainer, startEstimate);
    }

    poseInfoContainer_.update (slamProcessor->getLastScanMatchPose (),
                               slamProcessor->getLastScanMatchCovariance (),
                               laser->header.stamp, "map");

    /*
    if (laserScanToDataContainer(*laser, laserScanContainer, slamProcessor->getScaleToMap()))
    {
      slamProcessor->update(laserScanContainer,slamProcessor->getLastScanMatchPose());
    }
    */

    /*
    NS_NaviCommon::console.debug (
          "Pose update : %f, %f...",
          poseInfoContainer_.getPoseStamped ().pose.position.x,
          poseInfoContainer_.getPoseStamped ().pose.position.y);
          */

    /*
     * process map->odom transform
     */
    NS_Transform::StampedTransform odom_to_base;
    NS_ServiceType::RequestTransform request_odom;
    NS_ServiceType::ResponseTransform odom_transform;
    
    if (service->call (NS_NaviCommon::SERVICE_TYPE_ODOMETRY_BASE_TRANSFORM,
                       &request_odom, &odom_transform))
    {
      boost::mutex::scoped_lock map_mutex (map_to_odom_lock);
      map_to_odom = NS_Transform::Transform (
          poseInfoContainer_.getTfTransform () * odom_to_base.inverse ());
    }
    
    delete laser_data;
  }
  
  void
  HectorMappingApplication::mapService (NS_ServiceType::RequestBase* request,
                                        NS_ServiceType::ResponseBase* response)
  {
    NS_ServiceType::RequestMap* req = (NS_ServiceType::RequestMap*) request;
    NS_ServiceType::ResponseMap* rep = (NS_ServiceType::ResponseMap*) response;
    
    boost::mutex::scoped_lock map_mutex (map_lock);
    if (map.info.width && map.info.height)
    {
      rep->map = map;
      rep->result = true;
    }
    else
    {
      NS_NaviCommon::console.warning ("Get map failure!");
      rep->result = false;
    }
  }
  
  void
  HectorMappingApplication::mapTransformService (
      NS_ServiceType::RequestBase* request,
      NS_ServiceType::ResponseBase* response)
  {
    NS_ServiceType::RequestTransform* req =
        (NS_ServiceType::RequestTransform*) request;
    NS_ServiceType::ResponseTransform* rep =
        (NS_ServiceType::ResponseTransform*) response;
    
    boost::mutex::scoped_lock map_mutex (map_to_odom_lock);
    transformTFToMsg (map_to_odom, rep->transform);
  }
  
  void
  HectorMappingApplication::getMapInfo (
      NS_DataType::OccupancyGrid& map, const NS_HectorMapping::GridMap& gridMap)
  {
    Eigen::Vector2f mapOrigin (
        gridMap.getWorldCoords (Eigen::Vector2f::Zero ()));
    mapOrigin.array () -= gridMap.getCellLength () * 0.5f;
    
    map.info.origin.position.x = mapOrigin.x ();
    map.info.origin.position.y = mapOrigin.y ();
    map.info.origin.orientation.w = 1.0;
    
    map.info.resolution = gridMap.getCellLength ();
    
    map.info.width = gridMap.getSizeX ();
    map.info.height = gridMap.getSizeY ();
    
    map.header.frame_id = "map";
    map.data.resize (map.info.width * map.info.height);
  }
  
  const boost::numeric::ublas::matrix<double>&
  HectorMappingApplication::getUnitVectors (double angle_min, double angle_max,
                                            double angle_increment,
                                            unsigned int length)
  {
    boost::mutex::scoped_lock guv_lock (guv_mutex);
    
    //construct string for lookup in the map
    std::stringstream anglestring;
    anglestring << angle_min << "," << angle_max << "," << angle_increment
        << "," << length;
    std::map<std::string, boost::numeric::ublas::matrix<double>*>::iterator it;
    it = unit_vector_map.find (anglestring.str ());
    //check the map for presense
    if (it != unit_vector_map.end ())
      return *((*it).second);     //if present return
      
    boost::numeric::ublas::matrix<double> * tempPtr =
        new boost::numeric::ublas::matrix<double> (2, length);
    for (unsigned int index = 0; index < length; index++)
    {
      (*tempPtr) (0, index) = cos (
          angle_min + (double) index * angle_increment);
      (*tempPtr) (1, index) = sin (
          angle_min + (double) index * angle_increment);
    }
    //store
    unit_vector_map[anglestring.str ()] = tempPtr;
    //and return
    return *tempPtr;
  }
  ;
  
  void
  HectorMappingApplication::projectLaser (
      const NS_DataType::LaserScan& scan_in,
      std::vector<NS_DataType::Point32>& points, double range_cutoff,
      bool preservative)
  {
    boost::numeric::ublas::matrix<double> ranges (2, scan_in.ranges.size ());
    
    // Fill the ranges matrix
    for (unsigned int index = 0; index < scan_in.ranges.size (); index++)
    {
      ranges (0, index) = (double) scan_in.ranges[index];
      ranges (1, index) = (double) scan_in.ranges[index];
    }
    
    //Do the projection
    //    NEWMAT::Matrix output = NEWMAT::SP(ranges, getUnitVectors(scan_in.angle_min, scan_in.angle_max, scan_in.angle_increment));
    boost::numeric::ublas::matrix<double> output = element_prod (
        ranges,
        getUnitVectors (scan_in.angle_min, scan_in.angle_max,
                        scan_in.angle_increment, scan_in.ranges.size ()));
    
    //Stuff the output cloud
    points.resize (scan_in.ranges.size ());
    
    if (range_cutoff < 0)
      range_cutoff = scan_in.range_max;
    
    unsigned int count = 0;
    for (unsigned int index = 0; index < scan_in.ranges.size (); index++)
    {
      const float range = ranges (0, index);
      if (preservative
          || ((range < range_cutoff) && (range >= scan_in.range_min))) //if valid or preservative
      {
        points[count].x = output (0, index);
        points[count].y = output (1, index);
        points[count].z = 0.0;
        
        count++;
      }
    }
    
    //downsize if necessary
    points.resize (count);
  }
  ;
  
  bool
  HectorMappingApplication::pointsToDataContainer (
      const std::vector<NS_DataType::Point32>& points,
      const NS_Transform::StampedTransform& laserTransform,
      DataContainer& dataContainer, float scaleToMap)
  {
    size_t size = points.size ();
    //ROS_INFO("size: %d", size);
    
    dataContainer.clear ();
    
    NS_Transform::Vector3 laserPos (laserTransform.getOrigin ());
    dataContainer.setOrigo (Eigen::Vector2f (laserPos.x (), laserPos.y ()) * scaleToMap);
    
    for (size_t i = 0; i < size; ++i)
    {
      
      const NS_DataType::Point32& currPoint (points[i]);

      float dist_sqr = currPoint.x * currPoint.x + currPoint.y * currPoint.y;
      
      if ((dist_sqr > sqr_laser_min_dist_) && (dist_sqr < sqr_laser_max_dist_))
      {
        
        if ((currPoint.x < 0.0f) && (dist_sqr < 0.50f))
        {
          continue;
        }
        

        NS_Transform::Vector3 pointPosBaseFrame (
            laserTransform * NS_Transform::Vector3 (currPoint.x, currPoint.y,
                                         currPoint.z));
        
        float pointPosLaserFrameZ = pointPosBaseFrame.z () - laserPos.z ();
        
        if (pointPosLaserFrameZ > laser_z_min_value_
            && pointPosLaserFrameZ < laser_z_max_value_)
        {
          dataContainer.add (
              Eigen::Vector2f (pointPosBaseFrame.x (), pointPosBaseFrame.y ())
                  * scaleToMap);

        }
      }
    }
    
    return true;
  }
  
  bool
  HectorMappingApplication::laserScanToDataContainer (
      const NS_DataType::LaserScan& scan, DataContainer& dataContainer,
      float scaleToMap)
  {
    size_t size = scan.ranges.size ();

    float angle = scan.angle_min;

    dataContainer.clear ();

    dataContainer.setOrigo (Eigen::Vector2f::Zero ());

    float maxRangeForContainer = scan.range_max - 0.1f;

    for (size_t i = 0; i < size; ++i)
    {
      float dist = scan.ranges[i];

      if ((dist > scan.range_min) && (dist < maxRangeForContainer))
      {
        dist *= scaleToMap;
        dataContainer.add (
            Eigen::Vector2f (cos (angle) * dist, sin (angle) * dist));
      }

      angle += scan.angle_increment;
    }

    return true;
  }

  void
  HectorMappingApplication::updateMap (NS_DataType::OccupancyGrid& map,
                                       const NS_HectorMapping::GridMap& gridMap,
                                       MapLockerInterface* mapMutex)
  {
    //only update map if it changed
    if (lastGetMapUpdateIndex != gridMap.getUpdateIndex ())
    {
      
      int sizeX = gridMap.getSizeX ();
      int sizeY = gridMap.getSizeY ();
      
      int size = sizeX * sizeY;
      
      map.info.width = sizeX;
      map.info.height = sizeY;

      map.data.clear ();
      
      map.data.resize (size);
      
      //std::vector contents are guaranteed to be contiguous, use memset to set all to unknown to save time in loop
      memset (&map.data[0], -1, sizeof(int8_t) * size);
      
      if (mapMutex)
      {
        mapMutex->lockMap ();
      }
      
      for (int i = 0; i < size; ++i)
      {
        if (gridMap.isFree (i))
        {
          map.data[i] = 0;
        }
        else if (gridMap.isOccupied (i))
        {
          map.data[i] = 100;
        }
      }
      
      lastGetMapUpdateIndex = gridMap.getUpdateIndex ();
      
      if (mapMutex)
      {
        mapMutex->unlockMap ();
      }
    }
  }
  
  void
  HectorMappingApplication::updateMapLoop (double frequency)
  {
    NS_NaviCommon::Rate r (frequency);
    while (running)
    {
      {
        boost::mutex::scoped_lock map_mutex (map_lock);
        
        updateMap (map, slamProcessor->getGridMap (0),
                   slamProcessor->getMapMutex (0));
      }
      
      r.sleep ();
    }
  }
  
  void
  HectorMappingApplication::loadParameters ()
  {
    parameter.loadConfigurationFile ("hector_mapping.xml");
    
    map_resolution_ = parameter.getParameter ("map_resolution", 0.025f);
    map_size_ = parameter.getParameter ("map_size", 1024);
    map_start_x_ = parameter.getParameter ("map_start_x", 0.5f);
    map_start_y_ = parameter.getParameter ("map_start_y", 0.5f);
    map_multi_res_levels_ = parameter.getParameter ("map_multi_res_levels", 1);
    update_factor_free_ = parameter.getParameter ("update_factor_free", 0.4f);
    update_factor_occupied_ = parameter.getParameter ("update_factor_occupied",
                                                      0.9f);
    map_update_distance_threshold_ = parameter.getParameter (
        "map_update_distance_threshold", 0.4f);
    map_update_angle_threshold_ = parameter.getParameter (
        "map_update_angle_threshold", 0.9f);
    
    double tmp = 0.0;
    
    tmp = parameter.getParameter ("laser_min_dist", 0.1f);
    
    sqr_laser_min_dist_ = static_cast<float> (tmp * tmp);
    
    tmp = parameter.getParameter ("laser_max_dist", 30.0f);
    sqr_laser_max_dist_ = static_cast<float> (tmp * tmp);
    
    tmp = parameter.getParameter ("laser_z_min_value", -1.0f);
    laser_z_min_value_ = static_cast<float> (tmp);
    
    tmp = parameter.getParameter ("laser_z_max_value", 1.0f);
    laser_z_max_value_ = static_cast<float> (tmp);
    
    map_update_frequency_ = parameter.getParameter ("map_update_frequency",
                                                    2.0f);
  }
  
  void
  HectorMappingApplication::initialize ()
  {
    NS_NaviCommon::console.message ("hector mapping is initializing!");
    
    loadParameters ();
    
    slamProcessor = new HectorSlamProcessor (
        static_cast<float> (map_resolution_), map_size_, map_size_,
        Eigen::Vector2f (map_start_x_, map_start_y_), map_multi_res_levels_,
        hectorDrawings, debugInfoProvider);
    
    slamProcessor->setUpdateFactorFree (update_factor_free_);
    slamProcessor->setUpdateFactorOccupied (update_factor_occupied_);
    slamProcessor->setMapUpdateMinDistDiff (map_update_distance_threshold_);
    slamProcessor->setMapUpdateMinAngleDiff (map_update_angle_threshold_);
    
    int mapLevels = slamProcessor->getMapLevels ();
    
    printf ("level = %d\n", mapLevels);
    
    slamProcessor->addMapMutex (0, new HectorMapMutex ());
    
    getMapInfo (map, slamProcessor->getGridMap (0));
    
    map_to_odom.setIdentity ();
    
    dispitcher->subscribe (
        NS_NaviCommon::DATA_TYPE_LASER_SCAN,
        boost::bind (&HectorMappingApplication::laserDataCallback, this, _1));
    
    service->advertise (
        NS_NaviCommon::SERVICE_TYPE_MAP,
        boost::bind (&HectorMappingApplication::mapService, this, _1, _2));
    
    service->advertise (
        NS_NaviCommon::SERVICE_TYPE_MAP_ODOMETRY_TRANSFORM,
        boost::bind (&HectorMappingApplication::mapTransformService, this, _1, _2));

    initialized = true;
  }
  
  void
  HectorMappingApplication::run ()
  {
    NS_NaviCommon::console.message ("hector is running!");
    running = true;
    update_map_thread = boost::thread (
        boost::bind (&HectorMappingApplication::updateMapLoop, this,
                     map_update_frequency_));
  }
  
  void
  HectorMappingApplication::quit ()
  {
    NS_NaviCommon::console.message ("hector is quitting!");
    running = false;
    update_map_thread.join ();
  }

} /* namespace NS_GMapping */
