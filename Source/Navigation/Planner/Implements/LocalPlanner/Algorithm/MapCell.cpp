#include "MapCell.h"

namespace NS_Planner{

  MapCell::MapCell()
    : cx(0), cy(0),
      target_dist(DBL_MAX),
      target_mark(false),
      within_robot(false)
  {}

  MapCell::MapCell(const MapCell& mc)
    : cx(mc.cx), cy(mc.cy),
      target_dist(mc.target_dist),
      target_mark(mc.target_mark),
      within_robot(mc.within_robot)
  {}
};
