#include "Layer.h"

namespace NS_CostMap
{
  
  Layer::Layer ()
      : layered_costmap_ (NULL), current_ (false), enabled_ (false),
          dispitcher_ (
          NULL), service_ (NULL)
  {
  }
  
  void
  Layer::initialize (LayeredCostmap* parent,
                     NS_NaviCommon::Dispitcher* dispitcher,
                     NS_NaviCommon::Service* service)
  {
    layered_costmap_ = parent;
    dispitcher_ = dispitcher;
    service_ = service;
    onInitialize ();
  }
  
  const std::vector<NS_DataType::Point>&
  Layer::getFootprint () const
  {
    return layered_costmap_->getFootprint ();
  }

}  // end namespace costmap_2d
