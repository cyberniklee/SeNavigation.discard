#ifndef _BASE_LOCAL_PLANNER_MAP_GRID_H_
#define _BASE_LOCAL_PLANNER_MAP_GRID_H_

#include <vector>
#include <iostream>
#include "MapCell.h"
#include "../../../../CostMap/CostMap2D/CostMap2D.h"
#include <DataSet/DataType/PoseStamped.h>
#include "MapCell.h"
#include "TrajectoryInc.h"

namespace NS_Planner
{
  /**
   * @class MapGrid
   * @brief A grid of MapCell cells that is used to propagate path and goal distances for the trajectory controller.
   */
  class MapGrid
  {
  public:
    /**
     * @brief  Creates a 0x0 map by default
     */
    MapGrid ();

    /**
     * @brief  Creates a map of size_x by size_y
     * @param size_x The width of the map 
     * @param size_y The height of the map 
     */
    MapGrid (unsigned int size_x, unsigned int size_y);

    /**
     * @brief  Returns a map cell accessed by (col, row)
     * @param x The x coordinate of the cell 
     * @param y The y coordinate of the cell 
     * @return A reference to the desired cell
     */
    inline MapCell&
    operator() (unsigned int x, unsigned int y)
    {
      return map_[size_x_ * y + x];
    }
    
    /**
     * @brief  Returns a map cell accessed by (col, row)
     * @param x The x coordinate of the cell 
     * @param y The y coordinate of the cell 
     * @return A copy of the desired cell
     */
    inline MapCell
    operator() (unsigned int x, unsigned int y) const
    {
      return map_[size_x_ * y + x];
    }
    
    inline MapCell&
    getCell (unsigned int x, unsigned int y)
    {
      return map_[size_x_ * y + x];
    }
    
    /**
     * @brief  Destructor for a MapGrid
     */
    ~MapGrid ()
    {
    }
    
    /**
     * @brief  Copy constructor for a MapGrid
     * @param mg The MapGrid to copy 
     */
    MapGrid (const MapGrid& mg);

    /**
     * @brief  Assignment operator for a MapGrid
     * @param mg The MapGrid to assign from 
     */
    MapGrid&
    operator= (const MapGrid& mg);

    /**
     * @brief reset path distance fields for all cells
     */
    void
    resetPathDist ();

    /**
     * @brief  check if we need to resize
     * @param size_x The desired width
     * @param size_y The desired height
     */
    void
    sizeCheck (unsigned int size_x, unsigned int size_y);

    /**
     * @brief Utility to share initialization code across constructors
     */
    void
    commonInit ();

    /**
     * @brief  Returns a 1D index into the MapCell array for a 2D index
     * @param x The desired x coordinate
     * @param y The desired y coordinate
     * @return The associated 1D index 
     */
    size_t
    getIndex (int x, int y);

    /**
     * return a value that indicates cell is in obstacle
     */
    inline double
    obstacleCosts ()
    {
      return map_.size ();
    }
    
    /**
     * returns a value indicating cell was not reached by wavefront
     * propagation of set cells. (is behind walls, regarding the region covered by grid)
     */
    inline double
    unreachableCellCosts ()
    {
      return map_.size () + 1;
    }
    
    /**
     * @brief  Used to update the distance of a cell in path distance computation
     * @param  current_cell The cell we're currently in 
     * @param  check_cell The cell to be updated
     */
    inline bool
    updatePathCell (MapCell* current_cell, MapCell* check_cell,
                    const NS_CostMap::Costmap2D& costmap);

    /**
     * increase global plan resolution to match that of the costmap by adding points linearly between global plan points
     * This is necessary where global planners produce plans with few points.
     * @param global_plan_in input
     * @param global_plan_output output
     * @param resolution desired distance between waypoints
     */
    static void
    adjustPlanResolution (
        const std::vector<NS_DataType::PoseStamped>& global_plan_in,
        std::vector<NS_DataType::PoseStamped>& global_plan_out,
        double resolution);

    /**
     * @brief  Compute the distance from each cell in the local map grid to the planned path
     * @param dist_queue A queue of the initial cells on the path 
     */
    void
    computeTargetDistance (std::queue<MapCell*>& dist_queue,
                           const NS_CostMap::Costmap2D& costmap);

    /**
     * @brief  Compute the distance from each cell in the local map grid to the local goal point
     * @param goal_queue A queue containing the local goal cell 
     */
    void
    computeGoalDistance (std::queue<MapCell*>& dist_queue,
                         const NS_CostMap::Costmap2D& costmap);

    /**
     * @brief Update what cells are considered path based on the global plan 
     */
    void
    setTargetCells (const NS_CostMap::Costmap2D& costmap,
                    const std::vector<NS_DataType::PoseStamped>& global_plan);

    /**
     * @brief Update what cell is considered the next local goal
     */
    void
    setLocalGoal (const NS_CostMap::Costmap2D& costmap,
                  const std::vector<NS_DataType::PoseStamped>& global_plan);

    double goal_x_, goal_y_; /**< @brief The goal distance was last computed from */
    
    unsigned int size_x_, size_y_; ///< @brief The dimensions of the grid
        
  private:
    
    std::vector<MapCell> map_; ///< @brief Storage for the MapCells
    
  };
}
;

#endif
