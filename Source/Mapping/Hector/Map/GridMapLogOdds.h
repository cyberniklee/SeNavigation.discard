#ifndef _GridMapLogOdds_h_
#define _GridMapLogOdds_h_

#include <cmath>

/**
 * Provides a log odds of occupancy probability representation for cells in a occupancy grid map.
 */
class LogOddsCell
{
public:
  
  /*
   void setOccupied()
   {
   logOddsVal += 0.7f;
   };

   void setFree()
   {
   logOddsVal -= 0.4f;
   };
   */

  /**
   * Sets the cell value to val.
   * @param val The log odds value.
   */
  void
  set (float val)
  {
    logOddsVal = val;
  }
  
  /**
   * Returns the value of the cell.
   * @return The log odds value.
   */
  float
  getValue () const
  {
    return logOddsVal;
  }
  
  /**
   * Returns wether the cell is occupied.
   * @return Cell is occupied
   */
  bool
  isOccupied () const
  {
    return logOddsVal > 0.0f;
  }
  
  bool
  isFree () const
  {
    return logOddsVal < 0.0f;
  }
  
  /**
   * Reset Cell to prior probability.
   */
  void
  resetGridCell ()
  {
    logOddsVal = 0.0f;
    updateIndex = -1;
  }
  
  //protected:
  
public:
  
  float logOddsVal; ///< The log odds representation of occupancy probability.
  int updateIndex;
  
};

/**
 * Provides functions related to a log odds of occupancy probability respresentation for cells in a occupancy grid map.
 */
class GridMapLogOddsFunctions
{
public:
  
  /**
   * Constructor, sets parameters like free and occupied log odds ratios.
   */
  GridMapLogOddsFunctions ()
  {
    this->setUpdateFreeFactor (0.4f);
    this->setUpdateOccupiedFactor (0.6f);
    /*
     //float probOccupied = 0.6f;
     float probOccupied = 0.9f;
     float oddsOccupied = probOccupied / (1.0f - probOccupied);
     logOddsOccupied = log(oddsOccupied);

     float probFree = 0.4f;
     float oddsFree = probFree / (1.0f - probFree);
     logOddsFree = log(oddsFree);
     */
  }
  
  /**
   * Update cell as occupied
   * @param cell The cell.
   */
  void
  updateSetOccupied (LogOddsCell& cell) const
  {
    if (cell.logOddsVal < 50.0f)
    {
      cell.logOddsVal += logOddsOccupied;
    }
  }
  
  /**
   * Update cell as free
   * @param cell The cell.
   */
  void
  updateSetFree (LogOddsCell& cell) const
  {
    
    cell.logOddsVal += logOddsFree;
    
  }
  
  void
  updateUnsetFree (LogOddsCell& cell) const
  {
    cell.logOddsVal -= logOddsFree;
  }
  
  /**
   * Get the probability value represented by the grid cell.
   * @param cell The cell.
   * @return The probability
   */
  float
  getGridProbability (const LogOddsCell& cell) const
  {
    float odds = exp (cell.logOddsVal);
    return odds / (odds + 1.0f);
    
    /*
     float val = cell.logOddsVal;

     //prevent #IND when doing exp(large number).
     if (val > 50.0f) {
     return 1.0f;
     } else {
     float odds = exp(val);
     return odds / (odds + 1.0f);
     }
     */
    //return 0.5f;
  }
  
  //void getGridValueMap( const LogOddsCell& cell) const{};
  //void isOccupied(LogOddsCell& cell) {};
  
  //void resetGridCell() {};
  
  void
  setUpdateFreeFactor (float factor)
  {
    logOddsFree = probToLogOdds (factor);
  }
  
  void
  setUpdateOccupiedFactor (float factor)
  {
    logOddsOccupied = probToLogOdds (factor);
  }
  
protected:
  
  float
  probToLogOdds (float prob)
  {
    float odds = prob / (1.0f - prob);
    return log (odds);
  }
  
  float logOddsOccupied; /// < The log odds representation of probability used for updating cells as occupied
  float logOddsFree; /// < The log odds representation of probability used for updating cells as free
  
};

#endif
