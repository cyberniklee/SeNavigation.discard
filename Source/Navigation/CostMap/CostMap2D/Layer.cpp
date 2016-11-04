#include "Layer.h"

namespace NS_CostMap
{

Layer::Layer()
  : layered_costmap_(NULL)
  , current_(false)
  , enabled_(false)
  , name_()
{}

void Layer::initialize(LayeredCostmap* parent, std::string name)
{
  layered_costmap_ = parent;
  name_ = name;
  onInitialize();
}

const std::vector<NS_DataType::Point>& Layer::getFootprint() const
{
  return layered_costmap_->getFootprint();
}

}  // end namespace costmap_2d
