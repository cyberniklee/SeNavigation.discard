#ifndef _GRID_PATH_H_
#define _GRID_PATH_H_
#include <vector>

#include "Traceback.h"

namespace NS_Planner
{
  
  class GridPath: public Traceback
  {
  public:
    GridPath (PotentialCalculator* p_calc)
        : Traceback (p_calc)
    {
    }
    bool
    getPath (float* potential, double start_x, double start_y, double end_x,
             double end_y, std::vector<std::pair<float, float> >& path);
  };

} //end namespace global_planner
#endif
