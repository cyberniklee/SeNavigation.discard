/*
 * GMappingApplication.cpp
 *
 *  Created on: 2016年10月11日
 *      Author: seeing
 */

#include "GMappingApplication.h"
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
#include "Utils/Stat.h"
#include <time.h>

#define DEG2RAD(x) ((x)*M_PI/180.)
#define RAD2DEG(x) ((x)*180./M_PI)

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace NS_GMapping
{
  
  GMappingApplication::GMappingApplication ()
  {
    
    map_to_odom = NS_Transform::Transform (
        NS_Transform::createQuaternionFromRPY (0, 0, 0),
        NS_Transform::Point (0, 0, 0));
    
    gsp = new NS_GMapping::GridSlamProcessor ();
    
    gsp_laser = NULL;
    gsp_odom = NULL;
    
    got_first_scan = false;
    got_map = false;
    
    seed_ = time (NULL);
  }
  
  GMappingApplication::~GMappingApplication ()
  {
    if (gsp)
      delete gsp;
    if (gsp_laser)
      delete gsp_laser;
    if (gsp_odom)
      delete gsp_odom;
  }
  
  void
  GMappingApplication::laserDataCallback (NS_DataType::DataBase* laser_data)
  {
    NS_DataType::LaserScan* laser = (NS_DataType::LaserScan*) laser_data;
    laser_count++;
///////////////////////////////////////////////////////////////////////////
/*
     for(int i = 0; i < laser->ranges.size(); i++)
     {
     float degree = RAD2DEG(laser->angle_min + laser->angle_increment * i);
     NS_NaviCommon::console.debug("--->   angle: %f, range: %f", degree, laser->ranges[i]);
     }
     return;
*/
////////////////////////////////////////////////////////////////////////////
    if (throttle_scans_ != 0)
    {
      if ((laser_count % throttle_scans_) != 0)
        return;
    }
    
    if (!got_first_scan)
    {
      if (!initMapper (*laser))
        return;
      got_first_scan = true;
    }
    
    OrientedPoint odom_pose;
    
    static NS_NaviCommon::Time last_map_update (0, 0);

    if (addScan (*laser, odom_pose))
    {
      NS_NaviCommon::console.message ("Add scan process..");
      OrientedPoint mpose =
          gsp->getParticles ()[gsp->getBestParticleIndex ()].pose;
      
      NS_Transform::Transform laser_to_map = NS_Transform::Transform (
          NS_Transform::createQuaternionFromRPY (0, 0, mpose.theta),
          NS_Transform::Vector3 (mpose.x, mpose.y, 0.0)).inverse ();
      NS_Transform::Transform odom_to_laser = NS_Transform::Transform (
          NS_Transform::createQuaternionFromRPY (0, 0, odom_pose.theta),
          NS_Transform::Vector3 (odom_pose.x, odom_pose.y, 0.0));
      NS_NaviCommon::console.debug ("new best pose: %.3f, %.3f, %.3f.", mpose.x,
                                    mpose.y, mpose.theta);
      NS_NaviCommon::console.debug ("odom pose: %.3f, %.3f, %.3f.", odom_pose.x,
                                    odom_pose.y, odom_pose.theta);
      NS_NaviCommon::console.debug ("correction: %.3f, %.3f, %.3f.",
                                    mpose.x - odom_pose.x,
                                    mpose.y - odom_pose.y,
                                    mpose.theta - odom_pose.theta);
      
      map_to_odom_lock.lock ();
      map_to_odom = (odom_to_laser * laser_to_map).inverse ();
      map_to_odom_lock.unlock ();

      if (!got_map
          || (laser->header.stamp - last_map_update) > map_update_interval_)
      {
        updateMap (*laser);
        last_map_update = laser->header.stamp;
        NS_NaviCommon::console.message ("Updated the map...");
      }
    }
    else
    {
      NS_NaviCommon::console.warning ("Can not process the scan!");
    }
  }
  

  void
  GMappingApplication::mapService (NS_ServiceType::RequestBase* request,
                                   NS_ServiceType::ResponseBase* response)
  {
    NS_ServiceType::RequestMap* req = (NS_ServiceType::RequestMap*) request;
    NS_ServiceType::ResponseMap* rep = (NS_ServiceType::ResponseMap*) response;

    boost::mutex::scoped_lock map_mutex (map_lock);
    if (got_map && map.info.width && map.info.height)
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
  GMappingApplication::mapTransformService (
      NS_ServiceType::RequestBase* request,
      NS_ServiceType::ResponseBase* response)
  {
    NS_ServiceType::RequestTransform* req =
        (NS_ServiceType::RequestTransform*) request;
    NS_ServiceType::ResponseTransform* rep =
        (NS_ServiceType::ResponseTransform*) response;
    
    boost::mutex::scoped_lock map_mutex (map_lock);
    transformTFToMsg (map_to_odom, rep->transform);
    
  }
  
  double
  GMappingApplication::computePoseEntropy ()
  {
    double weight_total = 0.0;
    for (std::vector<GridSlamProcessor::Particle>::const_iterator it =
        gsp->getParticles ().begin (); it != gsp->getParticles ().end (); ++it)
    {
      weight_total += it->weight;
    }
    double entropy = 0.0;
    for (std::vector<GridSlamProcessor::Particle>::const_iterator it =
        gsp->getParticles ().begin (); it != gsp->getParticles ().end (); ++it)
    {
      if (it->weight / weight_total > 0.0)
        entropy += it->weight / weight_total * log (it->weight / weight_total);
    }
    return -entropy;
  }
  
  bool
  GMappingApplication::getOdomPose (OrientedPoint& gmap_pose)
  {
    //TODO:transform from odom to laser
    
    NS_ServiceType::RequestTransform request_odom;
    NS_ServiceType::ResponseTransform odom_transform;
    
    if (service->call (NS_NaviCommon::SERVICE_TYPE_ODOMETRY_BASE_TRANSFORM,
                       &request_odom, &odom_transform) == false)
    {
      NS_NaviCommon::console.warning ("Get odometry transform failure!");
      return false;
    }
    
    double yaw = NS_Transform::getYaw (odom_transform.transform.rotation);
    
    gmap_pose = OrientedPoint (odom_transform.transform.translation.x,
                               odom_transform.transform.translation.y, yaw);
    
    return true;
  }

  bool
  GMappingApplication::initMapper (NS_DataType::LaserScan& laser_data)
  {
    gsp_laser_beam_count = laser_data.ranges.size ();
    
    double angle_center = (laser_data.angle_max + laser_data.angle_min) / 2;
    
    if (up_mounted)
    {
      do_reverse_range = laser_data.angle_min > laser_data.angle_max;
      centered_laser_pose = NS_Transform::Transform (
          NS_Transform::createQuaternionFromRPY (0, 0, angle_center),
          NS_Transform::Vector3 (0, 0, 0));
      NS_NaviCommon::console.message ("The laser scanner is mounted upwards");
    }
    else
    {
      do_reverse_range = laser_data.angle_min < laser_data.angle_max;
      centered_laser_pose = NS_Transform::Transform (
          NS_Transform::createQuaternionFromRPY (M_PI, 0, -angle_center),
          NS_Transform::Vector3 (0, 0, 0));
      NS_NaviCommon::console.message ("The laser scanner is mounted downwards");
    }
    
    laser_angles.resize (laser_data.ranges.size ());
    
    double theta = -std::fabs (laser_data.angle_min - laser_data.angle_max) / 2;
    for (unsigned int i = 0; i < laser_data.ranges.size (); ++i)
    {
      laser_angles[i] = theta;
      theta += std::fabs (laser_data.angle_increment);
    }
    
    NS_NaviCommon::console.debug (
        "Laser got first frame min:%.3f, max:%.3f, inc:%.3f",
        laser_data.angle_min, laser_data.angle_max, laser_data.angle_increment);
    
    OrientedPoint gmap_pose (0, 0, 0);
    
    gsp_laser = new RangeSensor ("FLASER", gsp_laser_beam_count,
                                 fabs (laser_data.angle_increment), gmap_pose,
                                 0.0, max_range_);
    if (gsp_laser == NULL)
    {
      NS_NaviCommon::console.error ("Initial laser sensor module failure!");
      return false;
    }
    
    SensorMap smap;
    smap.insert (std::make_pair (gsp_laser->getName (), gsp_laser));
    gsp->setSensorMap (smap);
    
    gsp_odom = new OdometrySensor ("ODOM");
    if (gsp_odom == NULL)
    {
      NS_NaviCommon::console.error ("Initial odom sensor module failure!");
      return false;
    }
    
    OrientedPoint initial_pose;
    if (!getOdomPose (initial_pose))
    {
      NS_NaviCommon::console.warning (
          "Unable to get initial pose, start with zero...");
      initial_pose = OrientedPoint (0.0, 0.0, 0.0);
    }
    
    gsp->setMatchingParameters (max_u_range_, max_range_, sigma_, kernel_size_,
                                lstep_, astep_, iterations_, lsigma_, ogain_,
                                lskip_);
    
    gsp->setMotionModelParameters (srr_, srt_, str_, stt_);
    
    gsp->setUpdateDistances (linear_update_, angular_update_,
                             resample_threshold_);
    
    gsp->setgenerateMap (false);
    
    gsp->GridSlamProcessor::init (particles_, xmin_, ymin_, xmax_, ymax_,
                                  delta_, initial_pose);
    
    gsp->setllsamplerange (llsamplerange_);
    
    gsp->setllsamplestep (llsamplestep_);
    
    gsp->setlasamplerange (lasamplerange_);
    
    gsp->setlasamplestep (lasamplestep_);
    
    gsp->setminimumScore (minimum_score_);
    
    sampleGaussian (1, seed_);
    
    NS_NaviCommon::console.message ("Mapper initialized!");
    
    return true;
  }
  
  void
  GMappingApplication::updateMap (NS_DataType::LaserScan& laser_data)
  {
    boost::mutex::scoped_lock map_mutex (map_lock);
    ScanMatcher matcher;
    
    matcher.setLaserParameters (laser_data.ranges.size (), &laser_angles[0],
                                gsp_laser->getPose ());
    matcher.setlaserMaxRange (max_range_);
    matcher.setusableRange (max_u_range_);
    matcher.setgenerateMap (true);

    GridSlamProcessor::Particle best =
        gsp->getParticles ()[gsp->getBestParticleIndex ()];
    /*
    double entropy = computePoseEntropy();
    NS_NaviCommon::console.debug ("Entropy : %f", entropy);
    */

    if (!got_map)
    {
      map.info.resolution = delta_;
      map.info.origin.position.x = 0.0;
      map.info.origin.position.y = 0.0;
      map.info.origin.position.z = 0.0;
      map.info.origin.orientation.x = 0.0;
      map.info.origin.orientation.y = 0.0;
      map.info.origin.orientation.z = 0.0;
      map.info.origin.orientation.w = 0.0;
    }
    
    Point center;
    center.x = (xmin_ + xmax_) / 2.0;
    center.y = (ymin_ + ymax_) / 2.0;
    
    NS_NaviCommon::console.debug ("Smap info :");

    NS_NaviCommon::console.debug ("  x [%.3f, %.3f] , y [%.3f, %.3f]:", xmin_, xmax_, ymin_, ymax_);

    NS_NaviCommon::console.debug ("  resolution [%.3f]:", delta_);

    ScanMatcherMap smap (center, xmin_, ymin_, xmax_, ymax_, delta_);
    
    NS_NaviCommon::console.debug ("Trajectory tree:");
    
    for (GridSlamProcessor::TNode* n = best.node; n; n = n->parent)
    {
      NS_NaviCommon::console.debug ("  %.3f  %.3f  %.3f", n->pose.x, n->pose.y,
                                    n->pose.theta);
      if (!n->reading)
      {
        NS_NaviCommon::console.warning ("Null node!");
        continue;
      }
      NS_NaviCommon::console.debug ("Processing node!");
      matcher.invalidateActiveArea ();
      matcher.computeActiveArea (smap, n->pose, &((*n->reading)[0]));
      matcher.setgenerateMap (true);
      matcher.registerScan (smap, n->pose, &((*n->reading)[0]));
    }
    
    if (map.info.width != (unsigned int) smap.getMapSizeX ()
        || map.info.height != (unsigned int) smap.getMapSizeY ())
    {
      Point wmin = smap.map2world (IntPoint (0, 0));
      Point wmax = smap.map2world (
          IntPoint (smap.getMapSizeX (), smap.getMapSizeY ()));
      xmin_ = wmin.x;
      ymin_ = wmin.y;
      xmax_ = wmax.x;
      ymax_ = wmax.y;
      
      NS_NaviCommon::console.message ("Map size update!current is %d x %d.",
                                      smap.getMapSizeX (), smap.getMapSizeY ());
      NS_NaviCommon::console.message (
          "Map position is from (%f, %f) to (%f, %f)", xmin_, ymin_, xmax_,
          ymax_);
      
      map.info.width = smap.getMapSizeX ();
      map.info.height = smap.getMapSizeY ();
      map.info.origin.position.x = xmin_;
      map.info.origin.position.y = ymin_;
      map.data.resize (map.info.width * map.info.height);
      
      NS_NaviCommon::console.message ("Map origin: (%f, %f).",
                                      map.info.origin.position.x,
                                      map.info.origin.position.y);
    }
    
    for (int x = 0; x < smap.getMapSizeX (); x++)
    {
      for (int y = 0; y < smap.getMapSizeY (); y++)
      {
        IntPoint p (x, y);
        double occ = smap.cell (p);
        assert(occ <= 1.0);

        if (occ < 0)
        {
          map.data[MAP_IDX(map.info.width, x, y)] = -1;
        }
        else if (occ > occ_thresh_)
        {
          map.data[MAP_IDX(map.info.width, x, y)] = 100;
        }
        else
        {
          map.data[MAP_IDX(map.info.width, x, y)] = 0;
        }
      }
    }
    
    got_map = true;
  }
  
  bool
  GMappingApplication::addScan (NS_DataType::LaserScan& laser_data,
                                OrientedPoint& gmap_pose)
  {
    if (!getOdomPose (gmap_pose))
    {
      NS_NaviCommon::console.debug ("Get odometry pose failure!");
      return false;
    }
    
    if (laser_data.ranges.size () != gsp_laser_beam_count)
    {
      NS_NaviCommon::console.debug ("Laser beam count number is not correct!");
      return false;
    }

    double* ranges_double = new double[laser_data.ranges.size ()];
    
    if (do_reverse_range)
    {
      NS_NaviCommon::console.message ("Inverting scan data!");
      int num_ranges = laser_data.ranges.size ();
      for (int i = 0; i < num_ranges; i++)
      {
        if (laser_data.ranges[num_ranges - i - 1] < laser_data.range_min)
        {
          ranges_double[i] = (double) laser_data.range_max;
        }
        else
        {
          ranges_double[i] = (double) laser_data.ranges[num_ranges - i - 1];
        }
      }
    }
    else
    {
      for (unsigned int i = 0; i < laser_data.ranges.size (); i++)
      {
        if (laser_data.ranges[i] < laser_data.range_min)
        {
          ranges_double[i] = (double) laser_data.range_max;
        }
        else
        {
          ranges_double[i] = (double) laser_data.ranges[i];
        }

      }
    }
    
    RangeReading reading (laser_data.ranges.size (), ranges_double, gsp_laser,
                          laser_data.header.stamp.toSec ());
    
    delete[] ranges_double;
    
    reading.setPose (gmap_pose);
    
    return gsp->processScan (reading);
  }
  
  void
  GMappingApplication::loadParameters ()
  {
    parameter.loadConfigurationFile ("gmapping.xml");
    
    if (parameter.getParameter ("up_mounted", 1) == 1)
      up_mounted = true;
    else up_mounted = false;
    
    max_range_ = parameter.getParameter ("max_range", 6.0f);
    max_u_range_ = parameter.getParameter ("max_u_range", 4.0f);
    minimum_score_ = parameter.getParameter ("minimum_score", 0.0f);
    sigma_ = parameter.getParameter ("sigma", 0.05f);
    kernel_size_ = parameter.getParameter ("kernel_size", 1);

    lstep_ = parameter.getParameter ("lstep", 0.05f);
    astep_ = parameter.getParameter ("astep", 0.05f);

    iterations_ = parameter.getParameter ("iterations", 5);
    lsigma_ = parameter.getParameter ("lsigma", 0.075f);
    ogain_ = parameter.getParameter ("ogain", 3.0f);
    lskip_ = parameter.getParameter ("lskip", 1);
    
    srr_ = parameter.getParameter ("srr", 0.1f);
    srt_ = parameter.getParameter ("srt", 0.2f);
    str_ = parameter.getParameter ("str", 0.1f);
    stt_ = parameter.getParameter ("stt", 0.2f);
    
    linear_update_ = parameter.getParameter ("linear_update", 0.0f);
    angular_update_ = parameter.getParameter ("angular_update", 0.0f);
    temporal_update_ = parameter.getParameter ("temporal_update", -1.0f);
    resample_threshold_ = parameter.getParameter ("resample_threshold", 0.5f);
    particles_ = parameter.getParameter ("particles", 30);
    
    xmin_ = parameter.getParameter ("xmin", -25.0f);
    ymin_ = parameter.getParameter ("ymin", -25.0f);
    xmax_ = parameter.getParameter ("xmax", 25.0f);
    ymax_ = parameter.getParameter ("ymax", 25.0f);
    delta_ = parameter.getParameter ("delta", 0.05f);
    
    occ_thresh_ = parameter.getParameter ("occ_thresh", 0.25f);
    
    llsamplerange_ = parameter.getParameter ("llsamplerange", 0.01f);
    llsamplestep_ = parameter.getParameter ("llsamplestep", 0.01f);
    lasamplerange_ = parameter.getParameter ("lasamplerange", 0.005f);
    lasamplestep_ = parameter.getParameter ("lasamplestep", 0.005f);
    
    throttle_scans_ = parameter.getParameter ("throttle_scans", 2);
    
    double map_update_interval_sec = parameter.getParameter (
        "map_update_interval", 1.0f);
    map_update_interval_ = NS_NaviCommon::Duration (map_update_interval_sec);
  }
  
  void
  GMappingApplication::initialize ()
  {
    NS_NaviCommon::console.message ("gmapping is initializing!");
    
    loadParameters ();
    
    dispitcher->subscribe (
        NS_NaviCommon::DATA_TYPE_LASER_SCAN,
        boost::bind (&GMappingApplication::laserDataCallback, this, _1));

    service->advertise (
        NS_NaviCommon::SERVICE_TYPE_MAP,
        boost::bind (&GMappingApplication::mapService, this, _1, _2));

    initialized = true;
  }
  
  void
  GMappingApplication::run ()
  {
    NS_NaviCommon::console.message ("gmapping is running!");
    running = true;
  }
  
  void
  GMappingApplication::quit ()
  {
    NS_NaviCommon::console.message ("gmapping is quitting!");
    running = false;
  }

} /* namespace NS_GMapping */
