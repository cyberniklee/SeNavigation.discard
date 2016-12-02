#ifndef _COSTMAP_STATIC_LAYER_H_
#define _COSTMAP_STATIC_LAYER_H_

#include "../CostMap2D/CostMapLayer.h"
#include "../CostMap2D/LayeredCostMap.h"
#include <DataSet/DataType/OccupancyGrid.h>
#include <DataSet/DataType/OccupancyGridUpdate.h>
#include <boost/thread/thread.hpp>

namespace NS_CostMap
{

class StaticLayer : public CostmapLayer
{
public:
  StaticLayer();
  virtual ~StaticLayer();
  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();
  virtual void reset();

  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual void matchSize();

private:

  unsigned char interpretValue(unsigned char value);

private:
  unsigned int x_, y_, width_, height_;
  bool track_unknown_space_;
  bool use_maximum_;
  bool first_map_only_;      ///< @brief Store the first static map and reuse it on reinitializing
  bool trinary_costmap_;

  unsigned char lethal_threshold_, unknown_cost_value_;

  double map_update_frequency_;

private:

  bool active;

  boost::thread static_layer_loop;

  bool map_received;

  bool has_updated_data;

  void loopStaticMap();

  void processMap(const NS_DataType::OccupancyGrid& new_map);

};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_STATIC_LAYER_H_
