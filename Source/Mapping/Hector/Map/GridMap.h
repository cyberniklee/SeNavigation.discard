#ifndef _GridMap_h_
#define _GridMap_h_

#include "../Map/GridMapLogOdds.h"
#include "../Map/GridMapReflectanceCount.h"
#include "../Map/GridMapSimpleCount.h"
#include "../Map/OccGridMapBase.h"

namespace NS_HectorMapping
{
  
  typedef OccGridMapBase<LogOddsCell, GridMapLogOddsFunctions> GridMap;
//typedef OccGridMapBase<SimpleCountCell, GridMapSimpleCountFunctions> GridMap;
//typedef OccGridMapBase<ReflectanceCell, GridMapReflectanceFunctions> GridMap;

}

#endif
