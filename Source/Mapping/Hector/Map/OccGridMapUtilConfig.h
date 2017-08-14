#ifndef _OccGridMapUtilConfig_h_
#define _OccGridMapUtilConfig_h_

#include "../Map/OccGridMapUtil.h"

//#define SLAM_USE_HASH_CACHING
#ifdef SLAM_USE_HASH_CACHING
#include "GridMapCacheHash.h"
typedef GridMapCacheHash GridMapCacheMethod;
#else
#include "../Map/GridMapCacheArray.h"
typedef GridMapCacheArray GridMapCacheMethod;
#endif

namespace NS_HectorMapping
{
  
  template<typename ConcreteOccGridMap>
    class OccGridMapUtilConfig: public OccGridMapUtil<ConcreteOccGridMap,
        GridMapCacheMethod>
    {
    public:
      
      OccGridMapUtilConfig (ConcreteOccGridMap* gridMap = 0)
          : OccGridMapUtil<ConcreteOccGridMap, GridMapCacheMethod> (gridMap)
      {
      }
    };

}

#endif
