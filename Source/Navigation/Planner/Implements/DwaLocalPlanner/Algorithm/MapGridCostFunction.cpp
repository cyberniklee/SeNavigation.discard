
#include "MapGridCostFunction.h"
#include <Console/Console.h>

namespace NS_Planner {

MapGridCostFunction::MapGridCostFunction(NS_CostMap::Costmap2D* costmap,
    double xshift,
    double yshift,
    bool is_local_goal_function,
    CostAggregationType aggregationType) :
    costmap_(costmap),
    map_(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()),
    aggregationType_(aggregationType),
    xshift_(xshift),
    yshift_(yshift),
    is_local_goal_function_(is_local_goal_function),
    stop_on_failure_(true) {}

void MapGridCostFunction::setTargetPoses(std::vector<NS_DataType::PoseStamped> target_poses) {
  target_poses_ = target_poses;
}

bool MapGridCostFunction::prepare() {
  map_.resetPathDist();

  if (is_local_goal_function_) {
    map_.setLocalGoal(*costmap_, target_poses_);
  } else {
    map_.setTargetCells(*costmap_, target_poses_);
  }
  return true;
}

double MapGridCostFunction::getCellCosts(unsigned int px, unsigned int py) {
  double grid_dist = map_(px, py).target_dist;
  return grid_dist;
}

double MapGridCostFunction::scoreTrajectory(Trajectory &traj) {
  double cost = 0.0;
  if (aggregationType_ == Product) {
    cost = 1.0;
  }
  double px, py, pth;
  unsigned int cell_x, cell_y;
  double grid_dist;

  for (unsigned int i = 0; i < traj.getPointsSize(); ++i) {
    traj.getPoint(i, px, py, pth);

    // translate point forward if specified
    if (xshift_ != 0.0) {
      px = px + xshift_ * cos(pth);
      py = py + xshift_ * sin(pth);
    }
    // translate point sideways if specified
    if (yshift_ != 0.0) {
      px = px + yshift_ * cos(pth + M_PI_2);
      py = py + yshift_ * sin(pth + M_PI_2);
    }

    //we won't allow trajectories that go off the map... shouldn't happen that often anyways
    if ( ! costmap_->worldToMap(px, py, cell_x, cell_y)) {
      //we're off the map
      NS_NaviCommon::console.warning("Off Map %f, %f", px, py);
      return -4.0;
    }
    grid_dist = getCellCosts(cell_x, cell_y);
    //if a point on this trajectory has no clear path to the goal... it may be invalid
    if (stop_on_failure_) {
      if (grid_dist == map_.obstacleCosts()) {
        return -3.0;
      } else if (grid_dist == map_.unreachableCellCosts()) {
        return -2.0;
      }
    }

    switch( aggregationType_ ) {
    case Last:
      cost = grid_dist;
      break;
    case Sum:
      cost += grid_dist;
      break;
    case Product:
      if (cost > 0) {
        cost *= grid_dist;
      }
      break;
    }
  }
  return cost;
}

} /* namespace base_local_planner */
