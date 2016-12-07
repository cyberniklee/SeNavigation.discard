/*
 * CostmapWrapper.cpp
 *
 *  Created on: 2016年11月3日
 *      Author: seeing
 */

#include "CostmapWrapper.h"
#include "Layers/StaticLayer.h"

#include <Console/Console.h>
#include <Service/ServiceType/RequestMap.h>
#include <Service/ServiceType/ResponseMap.h>
#include <Service/ServiceType/RequestTransform.h>
#include <Service/ServiceType/ResponseTransform.h>
#include <DataSet/DataType/PolygonStamped.h>
#include <Parameter/Parameter.h>
#include "Utils/Footprint.h"

namespace NS_CostMap {

CostmapWrapper::CostmapWrapper(NS_NaviCommon::Dispitcher* dispitcher_, NS_NaviCommon::Service* service_)
{
  dispitcher = dispitcher_;
  service = service_;

  layered_costmap = NULL;

  cost_translation_table = NULL;
  if (cost_translation_table == NULL)
  {
    cost_translation_table = new char[256];

    // special values:
    cost_translation_table[0] = 0;  // NO obstacle
    cost_translation_table[253] = 99;  // INSCRIBED obstacle
    cost_translation_table[254] = 100;  // LETHAL obstacle
    cost_translation_table[255] = -1;  // UNKNOWN

    // regular cost values scale the range 1 to 252 (inclusive) to fit
    // into 1 to 98 (inclusive).
    for (int i = 1; i < 253; i++)
    {
      cost_translation_table[ i ] = char(1 + (97 * (i - 1)) / 251);
    }
  }
}

CostmapWrapper::~CostmapWrapper()
{
  if(layered_costmap)
    delete layered_costmap;

  if(cost_translation_table)
    delete cost_translation_table;
}

void CostmapWrapper::loadLayers()
{
  StaticLayer* static_layer = new StaticLayer();
  layers.push_back(static_layer);

}

void CostmapWrapper::updateMap()
{
  // get global pose
  NS_Transform::Stamped < NS_Transform::Pose > pose;
  if (getRobotPose (pose))
  {
    double x = pose.getOrigin().x(),
           y = pose.getOrigin().y(),
           yaw = NS_Transform::getYaw(pose.getRotation());

    layered_costmap->updateMap(x, y, yaw);

    NS_DataType::PolygonStamped footprint;
    footprint.header.stamp = NS_NaviCommon::Time::now();
    transformFootprint(x, y, yaw, padded_footprint, footprint);
    setPaddedRobotFootprint(toPointVector(footprint.polygon));
  }
}

void CostmapWrapper::updateCostmap()
{
  Costmap2D* costmap_ = layered_costmap->getCostmap();
  double resolution = costmap_->getResolution();

  if (map.info.resolution != resolution ||
	  map.info.width != costmap_->getSizeInCellsX() ||
	  map.info.height != costmap_->getSizeInCellsY() ||
	  saved_origin_x != costmap_->getOriginX() ||
	  saved_origin_y != costmap_->getOriginY())
  {
	prepareMap();
  }
  /*
  else if (x0 < xn)
  {
	boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

	update.x = x0_;
	update.y = y0_;
	update.width = xn_ - x0_;
	update.height = yn_ - y0_;
	map.data.resize(update.width * update.height);

	unsigned int i = 0;
	for (unsigned int y = y0_; y < yn_; y++)
	{
	  for (unsigned int x = x0_; x < xn_; x++)
	  {
		unsigned char cost = costmap_->getCost(x, y);
		update.data[i++] = cost_translation_table_[ cost ];
	  }
	}
	costmap_update_pub_.publish(update);
  }
  */

  xn = yn = 0;
  x0 = costmap_->getSizeInCellsX();
  y0 = costmap_->getSizeInCellsY();
}

void CostmapWrapper::updateMapLoop(double frequency)
{
  NS_NaviCommon::Rate rate(frequency);
  while(running)
  {
    updateMap();
    if(layered_costmap->isInitialized())
    {
      unsigned int _x0_, _y0_, _xn_, _yn_;
      layered_costmap->getBounds(&_x0_, &_xn_, &_y0_, &_yn_);
      updateBounds(_x0_, _xn_, _y0_, _yn_);
      updateCostmap();
    }
    rate.sleep();
  }
}

void CostmapWrapper::loadParameters()
{
  NS_NaviCommon::Parameter parameter;
  parameter.loadConfigurationFile("costmap.xml");

  if(parameter.getParameter("track_unknown_space", 0) == 1)
    track_unknown_space_ = true;
  else track_unknown_space_ = false;

  footprint_ = parameter.getParameter("footprint", "[[0.16, 0.16], [0.16, -0.16], [-0.16, -0.16], [-0.16, 0.16]]");

  map_width_meters_ = parameter.getParameter("map_width", 6.0f);
  map_height_meters_ = parameter.getParameter("map_height", 6.0f);
  resolution_ = parameter.getParameter("resolution", 0.01f);

  map_update_frequency_ = parameter.getParameter("map_update_frequency", 5.0f);

  origin_x_ = 0.0;
  origin_y_ = 0.0;
}

void CostmapWrapper::prepareMap()
{
  Costmap2D* costmap_ = layered_costmap->getCostmap();

  boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  double resolution = costmap_->getResolution();

  map.header.stamp = NS_NaviCommon::Time::now();
  map.info.resolution = resolution;

  map.info.width = costmap_->getSizeInCellsX();
  map.info.height = costmap_->getSizeInCellsY();

  double wx, wy;
  costmap_->mapToWorld(0, 0, wx, wy);
  map.info.origin.position.x = wx - resolution / 2;
  map.info.origin.position.y = wy - resolution / 2;
  map.info.origin.position.z = 0.0;
  map.info.origin.orientation.w = 1.0;
  saved_origin_x = costmap_->getOriginX();
  saved_origin_y = costmap_->getOriginY();

  map.data.resize(map.info.width * map.info.height);

  unsigned char* data = costmap_->getCharMap();
  for (unsigned int i = 0; i < map.data.size(); i++)
  {
	map.data[i] = cost_translation_table[ data[ i ]];
  }
}

bool CostmapWrapper::getRobotPose(NS_Transform::Stamped<NS_Transform::Pose>& global_pose) const
{
  NS_ServiceType::RequestTransform request_odom;
  NS_ServiceType::ResponseTransform odom_transform;
  NS_ServiceType::ResponseTransform map_transform;

  if(service->call(NS_NaviCommon::SERVICE_TYPE_ODOMETRY_BASE_TRANSFORM, &request_odom, &odom_transform) == false)
  {
	NS_NaviCommon::console.warning("Get odometry transform failure!");
	return false;
  }

  if(service->call(NS_NaviCommon::SERVICE_TYPE_MAP_ODOMETRY_TRANSFORM, &request_odom, &map_transform) == false)
  {
  	NS_NaviCommon::console.warning("Get map transform failure!");
  	return false;
  }

  //TODO: not verify code for transform
  NS_Transform::Transform odom_tf, map_tf;
  NS_Transform::transformMsgToTF(odom_transform.transform, odom_tf);
  NS_Transform::transformMsgToTF(map_transform.transform, map_tf);

  global_pose.setData(odom_tf * map_tf);

  return true;
}

void CostmapWrapper::mapService(NS_ServiceType::RequestBase* request, NS_ServiceType::ResponseBase* response)
{
  NS_ServiceType::RequestMap* req = (NS_ServiceType::RequestMap*)request;
  NS_ServiceType::ResponseMap* rep = (NS_ServiceType::ResponseMap*)response;

  boost::mutex::scoped_lock map_mutex(map_lock);
  if(got_map && map.info.width && map.info.height)
  {
    rep->map = map;
    rep->result = true;
  }else{
    rep->result = false;
  }
}

void CostmapWrapper::setPaddedRobotFootprint(const std::vector<NS_DataType::Point>& points)
{
  padded_footprint = points;
  padFootprint(padded_footprint, footprint_padding_);

  layered_costmap->setFootprint(padded_footprint);
}

void CostmapWrapper::initialize()
{
  NS_NaviCommon::console.message("costmap is initializing!");
  loadLayers();
  loadParameters();

  layered_costmap = new LayeredCostmap(track_unknown_space_);

  CostMapLayersIterator it;
  for(it = layers.begin(); it != layers.end(); it++)
  {
	(*it)->initialize(layered_costmap, this->dispitcher, this->service);
  }

  xn = yn = 0;
  x0 = layered_costmap->getCostmap()->getSizeInCellsX();
  y0 = layered_costmap->getCostmap()->getSizeInCellsY();

  std::vector<NS_DataType::Point> footprint_from_param;
  if(!makeFootprintFromString(footprint_, footprint_from_param))
  {
    NS_NaviCommon::console.error("Footprint parameter parse failure!");
    return;
  }
  setPaddedRobotFootprint(footprint_from_param);

  layered_costmap->resizeMap((unsigned int)(map_width_meters_ / resolution_),
							  (unsigned int)(map_height_meters_ / resolution_),
							  resolution_, origin_x_, origin_y_);

}

void CostmapWrapper::start()
{
  NS_NaviCommon::console.message("costmap is running!");

  running = true;

  update_map_thread = boost::thread(boost::bind(&CostmapWrapper::updateMapLoop, this, map_update_frequency_));
}

void CostmapWrapper::stop()
{
  NS_NaviCommon::console.message("costmap is quitting!");
  running = false;
  update_map_thread.join();
}

} /* namespace NS_CostMap */
