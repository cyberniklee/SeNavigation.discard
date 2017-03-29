#include "Astar.h"
#include "../../../../CostMap/CostMap2D/CostValues.h"

namespace NS_Planner
{
  
  AStarExpansion::AStarExpansion (PotentialCalculator* p_calc, int xs, int ys)
      : Expander (p_calc, xs, ys)
  {
  }
  
  bool
  AStarExpansion::calculatePotentials (unsigned char* costs, double start_x,
                                       double start_y, double end_x,
                                       double end_y, int cycles,
                                       float* potential)
  {
    
    queue_.clear ();
    
    int start_i = toIndex (start_x, start_y);
    
    queue_.push_back (Index (start_i, 0));
    
    std::fill (potential, potential + ns_, POT_HIGH);
    
    potential[start_i] = 0;
    
    int goal_i = toIndex (end_x, end_y);
    int cycle = 0;
    
    while (queue_.size () > 0 && cycle < cycles)
    {
      Index top = queue_[0];
      
      std::pop_heap (queue_.begin (), queue_.end (), greater1 ());
      queue_.pop_back ();
      
      int i = top.i;
      if (i == goal_i)
        return true;
      
      add (costs, potential, potential[i], i + 1, end_x, end_y);
      add (costs, potential, potential[i], i - 1, end_x, end_y);
      add (costs, potential, potential[i], i + nx_, end_x, end_y);
      add (costs, potential, potential[i], i - nx_, end_x, end_y);
    }
    
    return false;
  }
  
  /* to DISCUSS */
  void
  AStarExpansion::add (unsigned char* costs, float* potential,
                       float prev_potential, int next_i, int end_x, int end_y)
  {
    if (potential[next_i] < POT_HIGH)
      return;
    
    if (costs[next_i] >= lethal_cost_
        && !(unknown_ && costs[next_i] == NS_CostMap::NO_INFORMATION))
      return;
    
    potential[next_i] = p_calc_->calculatePotential (
        potential, costs[next_i] + neutral_cost_, next_i, prev_potential);
    int x = next_i % nx_, y = next_i / nx_;
    float distance = abs (end_x - x) + abs (end_y - y);
    
    /*
     * Given a heap in the range [first,last-1), this function extends the range considered a heap to [first,last) by
     * placing the value in (last-1) into its corresponding location within it.
     */
    queue_.push_back (
        Index (next_i, potential[next_i] + distance * neutral_cost_));
    std::push_heap (queue_.begin (), queue_.end (), greater1 ());
  }

} //end namespace global_planner
