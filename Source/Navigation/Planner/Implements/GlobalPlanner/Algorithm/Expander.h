#ifndef _EXPANDER_H_
#define _EXPANDER_H_

#define POT_HIGH 1.0e10        // unassigned cell potential

#include "PotentialCalculator.h"

#include <Console/Console.h>

namespace NS_Planner
{
  
  class Expander
  {
  public:
    Expander (PotentialCalculator* p_calc, int nx, int ny)
        : unknown_ (true), lethal_cost_ (253), neutral_cost_ (50),
            factor_ (3.0), p_calc_ (p_calc)
    {
      setSize (nx, ny);
    }
    
    virtual bool
    calculatePotentials (unsigned char* costs, double start_x, double start_y,
                         double end_x, double end_y, int cycles,
                         float* potential) = 0;

    /**
     * @brief  Sets or resets the size of the map
     * @param nx The x size of the map
     * @param ny The y size of the map
     */
    virtual void
    setSize (int nx, int ny)
    {
      nx_ = nx;
      ny_ = ny;
      ns_ = nx * ny;
    } /**< sets or resets the size of the map */
    void
    setLethalCost (unsigned char lethal_cost)
    {
      lethal_cost_ = lethal_cost;
    }
    void
    setNeutralCost (unsigned char neutral_cost)
    {
      neutral_cost_ = neutral_cost;
    }
    void
    setFactor (float factor)
    {
      factor_ = factor;
    }
    void
    setHasUnknown (bool unknown)
    {
      unknown_ = unknown;
    }
    
    void
    clearEndpoint (unsigned char* costs, float* potential, int gx, int gy,
                   int s)
    {
      NS_NaviCommon::console.debug ("clear Endpoint running...");
      int startCell = toIndex (gx, gy);
      
      cout << "startCell = " << startCell << "; ";
      cout << "gx = " << gx << "; ";
      cout << "gy = " << gy << "; ";
      cout << "s = " << s << "\n";
      for (int i = -s; i <= s; i++)
      {
        for (int j = -s; j <= s; j++)
        {
          /*
           * TODO: n 有可能是负数，这样计算出来不经过检查作为数组下标可能导致崩溃
           */
          int n = startCell + i + nx_ * j;
          cout << "n = " << n << "\n";
          if (potential[n] < POT_HIGH)
            continue;
          float c = costs[n] + neutral_cost_;
          float pot = p_calc_->calculatePotential (potential, c, n);
          potential[n] = pot;
        }
      }
      NS_NaviCommon::console.debug ("Finish clearEndpoint...");
    }
    
  protected:
    inline int
    toIndex (int x, int y)
    {
      return x + nx_ * y;
    }
    
    /*  */
    int nx_, ny_, ns_; /**< size of grid, in pixels */
    bool unknown_;
    unsigned char lethal_cost_, neutral_cost_;
    int cells_visited_;
    float factor_;
    PotentialCalculator* p_calc_;
    
  };

} //end namespace global_planner
#endif
