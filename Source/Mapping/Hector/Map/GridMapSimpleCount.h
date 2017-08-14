#ifndef _GridMapSimpleCount_h_
#define _GridMapSimpleCount_h_

/**
 * Provides a (very) simple count based representation of occupancy
 */
class SimpleCountCell
{
public:
  
  /**
   * Sets the cell value to val.
   * @param val The log odds value.
   */
  void
  set (float val)
  {
    simpleOccVal = val;
  }
  
  /**
   * Returns the value of the cell.
   * @return The log odds value.
   */
  float
  getValue () const
  {
    return simpleOccVal;
  }
  
  /**
   * Returns wether the cell is occupied.
   * @return Cell is occupied
   */
  bool
  isOccupied () const
  {
    return (simpleOccVal > 0.5f);
  }
  
  bool
  isFree () const
  {
    return (simpleOccVal < 0.5f);
  }
  
  /**
   * Reset Cell to prior probability.
   */
  void
  resetGridCell ()
  {
    simpleOccVal = 0.5f;
    updateIndex = -1;
  }
  
//protected:
  
public:
  
  float simpleOccVal; ///< The log odds representation of occupancy probability.
  int updateIndex;
  
};

/**
 * Provides functions related to a log odds of occupancy probability respresentation for cells in a occupancy grid map.
 */
class GridMapSimpleCountFunctions
{
public:
  
  /**
   * Constructor, sets parameters like free and occupied log odds ratios.
   */
  GridMapSimpleCountFunctions ()
  {
    updateFreeVal = -0.10f;
    updateOccVal = 0.15f;
    
    updateFreeLimit = -updateFreeVal + updateFreeVal / 100.0f;
    updateOccLimit = 1.0f - (updateOccVal + updateOccVal / 100.0f);
  }
  
  /**
   * Update cell as occupied
   * @param cell The cell.
   */
  void
  updateSetOccupied (SimpleCountCell& cell) const
  {
    if (cell.simpleOccVal < updateOccLimit)
    {
      cell.simpleOccVal += updateOccVal;
    }
  }
  
  /**
   * Update cell as free
   * @param cell The cell.
   */
  void
  updateSetFree (SimpleCountCell& cell) const
  {
    if (cell.simpleOccVal > updateFreeLimit)
    {
      cell.simpleOccVal += updateFreeVal;
    }
  }
  
  void
  updateUnsetFree (SimpleCountCell& cell) const
  {
    cell.simpleOccVal -= updateFreeVal;
  }
  
  /**
   * Get the probability value represented by the grid cell.
   * @param cell The cell.
   * @return The probability
   */
  float
  getGridProbability (const SimpleCountCell& cell) const
  {
    return cell.simpleOccVal;
  }
  
protected:
  
  float updateFreeVal;
  float updateOccVal;

  float updateFreeLimit;
  float updateOccLimit;
  
};

#endif
