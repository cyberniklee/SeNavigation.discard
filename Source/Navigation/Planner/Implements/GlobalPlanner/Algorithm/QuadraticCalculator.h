#ifndef _QUADRATIC_CALCULATOR_H_
#define _QUADRATIC_CALCULATOR_H_
#include<vector>

#include "PotentialCalculator.h"

namespace NS_Planner
{
  
  class QuadraticCalculator: public PotentialCalculator
  {
  public:
    QuadraticCalculator (int nx, int ny)
        : PotentialCalculator (nx, ny)
    {
    }
    
    float
    calculatePotential (float* potential, unsigned char cost, int n,
                        float prev_potential);
  };

} //end namespace global_planner
#endif
