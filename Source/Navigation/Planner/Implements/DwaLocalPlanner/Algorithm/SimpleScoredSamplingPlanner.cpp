#include "SimpleScoredSamplingPlanner.h"

#include <Console/Console.h>

namespace NS_Planner
{
  
  SimpleScoredSamplingPlanner::SimpleScoredSamplingPlanner (
      std::vector<TrajectorySampleGenerator*> gen_list,
      std::vector<TrajectoryCostFunction*>& critics, int max_samples)
  {
    max_samples_ = max_samples;
    gen_list_ = gen_list;
    critics_ = critics;
  }
  
  double
  SimpleScoredSamplingPlanner::scoreTrajectory (Trajectory& traj,
                                                double best_traj_cost)
  {
    double traj_cost = 0;
    int gen_id = 0;
    for (std::vector<TrajectoryCostFunction*>::iterator score_function =
        critics_.begin (); score_function != critics_.end (); ++score_function)
    {
      TrajectoryCostFunction* score_function_p = *score_function;
      if (score_function_p->getScale () == 0)
      {
        continue;
      }
      double cost = score_function_p->scoreTrajectory (traj);
      if (cost < 0)
      {
        NS_NaviCommon::console.debug (
            "Velocity %.3lf, %.3lf, %.3lf discarded by cost function  %d with cost: %f",
            traj.xv_, traj.yv_, traj.thetav_, gen_id, cost);
        traj_cost = cost;
        break;
      }
      if (cost != 0)
      {
        cost *= score_function_p->getScale ();
      }
      traj_cost += cost;
      if (best_traj_cost > 0)
      {
        // since we keep adding positives, once we are worse than the best, we will stay worse
        if (traj_cost > best_traj_cost)
        {
          break;
        }
      }
      gen_id++;
    }
    
    return traj_cost;
  }
  
  bool
  SimpleScoredSamplingPlanner::findBestTrajectory (
      Trajectory& traj, std::vector<Trajectory>* all_explored)
  {
    Trajectory loop_traj;
    Trajectory best_traj;
    double loop_traj_cost, best_traj_cost = -1;
    bool gen_success;
    int count, count_valid;
    for (std::vector<TrajectoryCostFunction*>::iterator loop_critic =
        critics_.begin (); loop_critic != critics_.end (); ++loop_critic)
    {
      TrajectoryCostFunction* loop_critic_p = *loop_critic;
      if (loop_critic_p->prepare () == false)
      {
        NS_NaviCommon::console.warning ("A scoring function failed to prepare");
        return false;
      }
    }
    
    for (std::vector<TrajectorySampleGenerator*>::iterator loop_gen =
        gen_list_.begin (); loop_gen != gen_list_.end (); ++loop_gen)
    {
      count = 0;
      count_valid = 0;
      TrajectorySampleGenerator* gen_ = *loop_gen;
      while (gen_->hasMoreTrajectories ())
      {
        gen_success = gen_->nextTrajectory (loop_traj);
        if (gen_success == false)
        {
          // TODO use this for debugging
          continue;
        }
        loop_traj_cost = scoreTrajectory (loop_traj, best_traj_cost);
        if (all_explored != NULL)
        {
          loop_traj.cost_ = loop_traj_cost;
          all_explored->push_back (loop_traj);
        }
        
        if (loop_traj_cost >= 0)
        {
          count_valid++;
          if (best_traj_cost < 0 || loop_traj_cost < best_traj_cost)
          {
            best_traj_cost = loop_traj_cost;
            best_traj = loop_traj;
          }
        }
        count++;
        if (max_samples_ > 0 && count >= max_samples_)
        {
          break;
        }
      }
      if (best_traj_cost >= 0)
      {
        traj.xv_ = best_traj.xv_;
        traj.yv_ = best_traj.yv_;
        traj.thetav_ = best_traj.thetav_;
        traj.cost_ = best_traj_cost;
        traj.resetPoints ();
        double px, py, pth;
        for (unsigned int i = 0; i < best_traj.getPointsSize (); i++)
        {
          best_traj.getPoint (i, px, py, pth);
          traj.addPoint (px, py, pth);
        }
      }
      NS_NaviCommon::console.debug ("Evaluated %d trajectories, found %d valid",
                                    count, count_valid);
      if (best_traj_cost >= 0)
      {
        // do not try fallback generators
        break;
      }
    }
    return best_traj_cost >= 0;
  }

} // namespace
