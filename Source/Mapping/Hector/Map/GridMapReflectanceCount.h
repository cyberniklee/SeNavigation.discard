#ifndef _GridMapReflectanceCount_h_
#define _GridMapReflectanceCount_h_

/**
 * Provides a reflectance count representation for cells in a occupancy grid map.
 */
class ReflectanceCell
{
public:
  
  void
  set (float val)
  {
    probOccupied = val;
  }
  
  float
  getValue () const
  {
    return probOccupied;
  }
  
  bool
  isOccupied () const
  {
    return probOccupied > 0.5f;
  }
  
  bool
  isFree () const
  {
    return probOccupied < 0.5f;
  }
  
  void
  resetGridCell ()
  {
    probOccupied = 0.5f;
    visitedCount = 0.0f;
    reflectedCount = 0.0f;
    updateIndex = -1;
  }
  
//protected:
  
  float visitedCount;
  float reflectedCount;
  float probOccupied;
  int updateIndex;
};

class GridMapReflectanceFunctions
{
public:
  
  GridMapReflectanceFunctions ()
  {
  }
  
  void
  updateSetOccupied (ReflectanceCell& cell) const
  {
    ++cell.reflectedCount;
    ++cell.visitedCount;
    cell.probOccupied = cell.reflectedCount / cell.visitedCount;
  }
  
  void
  updateSetFree (ReflectanceCell& cell) const
  {
    ++cell.visitedCount;
    cell.probOccupied = cell.reflectedCount / cell.visitedCount;
  }
  
  void
  updateUnsetFree (ReflectanceCell& cell) const
  {
    --cell.visitedCount;
    cell.probOccupied = cell.reflectedCount / cell.visitedCount;
  }
  
  float
  getGridProbability (const ReflectanceCell& cell) const
  {
    return cell.probOccupied;
  }
  
protected:
  
};

#endif
