/*
 * CostmapWrapper.h
 *
 *  Created on: 2016年11月3日
 *      Author: seeing
 */

#ifndef _COSTMAP_COSTMAPWRAPPER_H_
#define _COSTMAP_COSTMAPWRAPPER_H_

#include <vector>
#include "CostMap2D/CostMapLayer.h"
#include <DataSet/DataType/OccupancyGrid.h>
#include <Service/ServiceType/RequestBase.h>
#include <Service/ServiceType/ResponseBase.h>
#include <DataSet/DataType/Point.h>
#include <Transform/DataTypes.h>
#include <DataSet/Dispitcher.h>
#include <Service/Service.h>

namespace NS_CostMap {

typedef std::vector<CostmapLayer*> CostMapLayers;
typedef std::vector<CostmapLayer*>::iterator CostMapLayersIterator;

class CostmapWrapper
{
public:
	CostmapWrapper(NS_NaviCommon::Dispitcher* dispitcher_, NS_NaviCommon::Service* service_);
	virtual ~CostmapWrapper();
private:
	bool track_unknown_space_;
	std::string footprint_;

	double map_update_frequency_;

	double map_width_meters_;
	double map_height_meters_;
	double resolution_;
	double origin_x_;
	double origin_y_;

	float footprint_padding_;
private:
	LayeredCostmap* layered_costmap;

	CostMapLayers layers;

	void updateMapLoop(double frequency);
	boost::thread update_map_thread;

	NS_DataType::OccupancyGrid map;
	boost::mutex map_lock;
	void mapService(NS_ServiceType::RequestBase* request, NS_ServiceType::ResponseBase* response);

	unsigned int x0, xn, y0, yn;
	double saved_origin_x, saved_origin_y;

	char* cost_translation_table;  ///< Translate from 0-255 values in costmap to -1 to 100 values in message.

	std::vector<NS_DataType::Point> padded_footprint;

	bool got_map;

	bool running;

private:
	void loadParameters();
	void loadLayers();

	void updateBounds(unsigned int x0_, unsigned int xn_, unsigned int y0_, unsigned int yn_)
	{
	  x0 = std::min(x0, x0_);
	  xn = std::max(xn, xn_);
	  y0 = std::min(y0, y0_);
	  yn = std::max(yn, yn_);
	}

	void prepareMap();

	void setPaddedRobotFootprint(const std::vector<NS_DataType::Point>& points);

	void updateMap();
	void updateCostmap();
public:
	LayeredCostmap* getLayeredCostmap()
	{
		return layered_costmap;
	};

	bool getRobotPose(NS_Transform::Stamped<NS_Transform::Pose>& global_pose) const;
public:
	void initialize();
	void start();
	void stop();
private:
	NS_NaviCommon::Dispitcher* dispitcher;
	NS_NaviCommon::Service* service;
};

} /* namespace NS_CostMap */

#endif /* NAVIGATION_COSTMAP_COSTMAPWRAPPER_H_ */
