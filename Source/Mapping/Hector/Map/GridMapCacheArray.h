#ifndef _GridMapCacheArray_h_
#define _GridMapCacheArray_h_

#include <Eigen/Core>

class CachedMapElement
{
public:
  float val;
  int index;
};

/**
 * Caches filtered grid map accesses in a two dimensional array of the same size as the map.
 */
class GridMapCacheArray
{
public:
  
  /**
   * Constructor
   */
  GridMapCacheArray ()
      : cacheArray (0), arrayDimensions (-1, -1)
  {
    currCacheIndex = 0;
  }
  
  /**
   * Destructor
   */
  ~GridMapCacheArray ()
  {
    deleteCacheArray ();
  }
  
  /**
   * Resets/deletes the cached data
   */
  void
  resetCache ()
  {
    currCacheIndex++;
  }
  
  /**
   * Checks wether cached data for coords is available. If this is the case, writes data into val.
   * @param coords The coordinates
   * @param val Reference to a float the data is written to if available
   * @return Indicates if cached data is available
   */
  bool
  containsCachedData (int index, float& val)
  {
    const CachedMapElement& elem (cacheArray[index]);
    
    if (elem.index == currCacheIndex)
    {
      val = elem.val;
      return true;
    }
    else
    {
      return false;
    }
  }
  
  /**
   * Caches float value val for coordinates coords.
   * @param coords The coordinates
   * @param val The value to be cached for coordinates.
   */
  void
  cacheData (int index, float val)
  {
    CachedMapElement& elem (cacheArray[index]);
    elem.index = currCacheIndex;
    elem.val = val;
  }
  
  /**
   * Sets the map size and resizes the cache array accordingly
   * @param sizeIn The map size.
   */
  void
  setMapSize (const Eigen::Vector2i& newDimensions)
  {
    setArraySize (newDimensions);
  }
  
protected:
  
  /**
   * Creates a cache array of size sizeIn.
   * @param sizeIn The size of the array
   */
  void
  createCacheArray (const Eigen::Vector2i& newDimensions)
  {
    arrayDimensions = newDimensions;
    
    int sizeX = arrayDimensions[0];
    int sizeY = arrayDimensions[1];
    
    int size = sizeX * sizeY;
    
    cacheArray = new CachedMapElement[size];
    
    for (int x = 0; x < size; ++x)
    {
      cacheArray[x].index = -1;
    }
  }
  
  /**
   * Deletes the existing cache array.
   */
  void
  deleteCacheArray ()
  {
    delete[] cacheArray;
  }
  
  /**
   * Sets a new cache array size
   */
  void
  setArraySize (const Eigen::Vector2i& newDimensions)
  {
    if (this->arrayDimensions != newDimensions)
    {
      if (cacheArray != 0)
      {
        deleteCacheArray ();
        cacheArray = 0;
      }
      createCacheArray (newDimensions);
    }
  }
  
protected:
  
  CachedMapElement* cacheArray;    ///< Array used for caching data.
  int currCacheIndex;              ///< The cache iteration index value
  
  Eigen::Vector2i arrayDimensions; ///< The size of the array
  
};

#endif
