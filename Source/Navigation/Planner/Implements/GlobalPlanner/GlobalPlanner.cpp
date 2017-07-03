/*
 * GlobalPlanner.cpp
 *
 *  Created on: 2016年12月2日
 *      Author: seeing
 */

#include "GlobalPlanner.h"

#include "Algorithm/QuadraticCalculator.h"

#include "Algorithm/GridPath.h"
#include "Algorithm/GradientPath.h"

#include "Algorithm/Astar.h"
#include "Algorithm/Dijkstra.h"

#include <DataSet/DataType/OccupancyGrid.h>
#include <Parameter/Parameter.h>

#include <Console/Console.h>

#include <iostream>
using namespace std;

/*
 * costmap 是用一维数组表示二维，通过 toIndex 来把二维数组中的坐标转化为一维数组中的位置
 */
namespace NS_Planner
{
  
  GlobalPlanner::GlobalPlanner ()
  {
    // TODO Auto-generated constructor stub
    
  }
  
  GlobalPlanner::~GlobalPlanner ()
  {
    // TODO Auto-generated destructor stub
  }
  
  /*
   * 把 costmap 代表的 4 个边上的点全部设为 value 值
   */
  void
  GlobalPlanner::outlineMap (unsigned char* costarr, int nx, int ny,
                             unsigned char value)
  {
    unsigned char* pc = costarr;
    for (int i = 0; i < nx; i++)
      *pc++ = value;
    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++)
      *pc++ = value;
    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx)
      *pc = value;
    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx)
      *pc = value;
  }
  
  void
  GlobalPlanner::onInitialize ()
  {
    if (!initialized_)
    {
//		NS_NaviCommon::console.debug("onInitialize running...");
      /*
       * 获取 costmap 的 size_x 和 size_y
       */
      unsigned int cx =
          costmap->getLayeredCostmap ()->getCostmap ()->getSizeInCellsX (), cy =
          costmap->getLayeredCostmap ()->getCostmap ()->getSizeInCellsY ();
      
      convert_offset_ = 0.5;
      
//		NS_NaviCommon::console.debug("After getting cx, cy...");
      
      NS_NaviCommon::Parameter parameter;
      /* TODO
       * 加载什么文件？
       */
      parameter.loadConfigurationFile ("global_planner.xml");
      /*
       * 获取 use_quadratic 参数值，根据参数值创建 p_calc_ 实例，用 QuadraticCalculator 还是 PotentialCalculator
       * PotentialCalculator、QuadraticCalculator
       */
//		NS_NaviCommon::console.debug("After loading...");
      if (parameter.getParameter ("use_quadratic", 1) == 1)
        p_calc_ = new QuadraticCalculator (cx, cy);
      else p_calc_ = new PotentialCalculator (cx, cy);
      
//		NS_NaviCommon::console.debug("After p_calc_ assignment...");
      /*
       * 获取 use_dijkstra 参数值，根据参数值创建 planner_ 实例，决定用 dijkstra 算法还是 A* 算法
       * Expander、Dijkstra、A*
       */
      if (parameter.getParameter ("use_dijkstra", 1) == 1)
      {
        DijkstraExpansion* de = new DijkstraExpansion (p_calc_, cx, cy);
        de->setPreciseStart (true);
        planner_ = de;
      }
      else
      {
        planner_ = new AStarExpansion (p_calc_, cx, cy);
      }
      
//		NS_NaviCommon::console.debug("After planner_ assignment");
      /*
       * 获取 use_grid_path 参数值，根据参数值创建 path_maker_ 实例，用 GridPath 还是 GradientPath
       * Traceback、GridPath、GradientPath
       */
      if (parameter.getParameter ("use_grid_path", 0) == 1)
        path_maker_ = new GridPath (p_calc_);
      else path_maker_ = new GradientPath (p_calc_);
      
//		NS_NaviCommon::console.debug("After path_maker_ assignment...");
      
      orientation_filter_ = new OrientationFilter ();
      
//		NS_NaviCommon::console.debug("orientation_filter_...");
      
//		由于 getParameter 方法没有提供返回值为 bool 类型的，因此用返回值为 int 的代替
      if (parameter.getParameter ("allow_unknown", 1) == 1)
      {
        allow_unknown_ = true;
      }
      else
      {
        allow_unknown_ = false;
      }
      
      planner_->setHasUnknown (allow_unknown_); // 该方法接收一个 bool 类型参数，所有非零值都作为 true
          
      planner_window_x_ = parameter.getParameter ("planner_window_x", 0.0f); // float 0.0f 指明调用参数为 float
      planner_window_y_ = parameter.getParameter ("planner_window_y", 0.0f);
      default_tolerance_ = parameter.getParameter ("default_tolerance", 0.0f);
      publish_scale_ = parameter.getParameter ("publish_scale", 100);
      
//		NS_NaviCommon::console.debug("A lot of parameters...");
      
//		NS_NaviCommon::console.debug("Finish onInitialize called");
      initialized_ = true;
    }
    else
    {
      NS_NaviCommon::console.error ("onInitialize has been called before");
    }
  }
  
  bool
  GlobalPlanner::makePlan (const NS_DataType::PoseStamped& start,
                           const NS_DataType::PoseStamped& goal,
                           std::vector<NS_DataType::PoseStamped>& plan)
  {
//	NS_NaviCommon::console.debug("GlobalPlanner makePlan running...");
    boost::mutex::scoped_lock lock (mutex_);
    
//	NS_NaviCommon::console.debug("After locking...");
    
    if (!initialized_)
    {
      NS_NaviCommon::console.error (
          "This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }
    
    //clear the plan, just in case
    // 先把 plan 清空
    plan.clear ();
    
    double wx = start.pose.position.x;
    double wy = start.pose.position.y;
    
//	cout << "start world wx = " << wx << " wy = " << wy << "\n";
    /*
     * 把现实位置映射到地图中
     * 获取 start 和 goal 在地图中的坐标
     */
    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;
    
    /*
     * costmap 是 CostmapWrapper*，
     * GlobalPlannerBase 的 protected 成员
     * CostmapWrapper 有一个私有成员 LayeredCostmap*，通过公共方法 getLayeredCostmap 获得
     * costmap -> getLayeredCostmap() 返回 LayeredCostmap*
     *
     * LayeredCostmap 有一个私有成员 Costmap2D，通过公共方法 getCostmap 获得
     *   Costmap2D* getCostmap()
     {
     return &costmap_;
     }
     */
    if (!costmap->getLayeredCostmap ()->getCostmap ()->worldToMap (wx, wy,
                                                                   start_x_i,
                                                                   start_y_i))
    {
      // 加一下错误提示
      NS_NaviCommon::console.error (
          "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
      return false;
    }
    //默认 old_navfn_behavior = false
    worldToMap (wx, wy, start_x, start_y);
    
//	cout << "start_x_i, start_y_i, start_x, start_y" <<
//			start_x_i << " " << start_y_i << " " << start_x << " " << start_y <<"\n";
    
    /*
     * 处理 goal
     */
    wx = goal.pose.position.x;
    wy = goal.pose.position.y;
    
//	cout << "goal world wx = " << wx << " wy = " << wy << "\n";
    
    if (!costmap->getLayeredCostmap ()->getCostmap ()->worldToMap (wx, wy,
                                                                   goal_x_i,
                                                                   goal_y_i))
    {
      // 加一下错误提示
      NS_NaviCommon::console.error (
          "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
      return false;
    }
    worldToMap (wx, wy, goal_x, goal_y);
    
//	cout << "goal_x_i, goal_y_i, goal_x, goal_y" <<
//				goal_x_i << " " << goal_y_i << " " << goal_x << " " << goal_y <<"\n";
    
//	NS_NaviCommon::console.debug("After worldToMap...");
    /*
     * 这个函数的第一个参数 const tf::Stamped<tf::Pose> 其实是没用上的，
     * 因此可以把函数签名去掉这个参数
     */
    //clear the starting cell within the costmap because we know it can't be an obstacle
//	tf::Stamped<tf::Pose> start_pose;
//	tf::poseStampedMsgToTF(start, start_pose);
//	clearRobotCell(start_pose, start_x_i, start_y_i);
    clearRobotCell (start_x_i, start_y_i);
    
//	NS_NaviCommon::console.debug("After clearRobotCell...");
    
    //int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
    
    int nx = costmap->getLayeredCostmap ()->getCostmap ()->getSizeInCellsX (),
        ny = costmap->getLayeredCostmap ()->getCostmap ()->getSizeInCellsY ();
    
//	NS_NaviCommon::console.debug("After getting nx, ny...");
//	cout << "nx, ny = " << nx << " " << ny << "\n";
    //make sure to resize the underlying array that Navfn uses
    p_calc_->setSize (nx, ny); // PotentialCalculator* p_calc_;
    planner_->setSize (nx, ny); // Expander* planner_;
    path_maker_->setSize (nx, ny); // Traceback* path_maker_;
    potential_array_ = new float[nx * ny]; // float* potential_array_;
        
//	NS_NaviCommon::console.debug("After parameters setSize...");
    
    outlineMap (costmap->getLayeredCostmap ()->getCostmap ()->getCharMap (), nx,
                ny, NS_CostMap::LETHAL_OBSTACLE);
    
//	NS_NaviCommon::console.debug("After outlineMap, invoking calculatePotentials...");
    /*
     * 此处开始调用算法
     */
    bool found_legal = planner_->calculatePotentials (
        costmap->getLayeredCostmap ()->getCostmap ()->getCharMap (), start_x,
        start_y, goal_x, goal_y, nx * ny * 2, potential_array_);
    
//	NS_NaviCommon::console.debug("After calculatePotentials, invoking clearEndPoint...");
    
    planner_->clearEndpoint (
        costmap->getLayeredCostmap ()->getCostmap ()->getCharMap (),
        potential_array_, goal_x_i, goal_y_i, 2);
    
//	NS_NaviCommon::console.debug("After clearEndpoint...");
    
    if (found_legal)
    {
      //extract the plan
      if (getPlanFromPotential (start_x, start_y, goal_x, goal_y, goal, plan))
      {
        //make sure the goal we push on has the same timestamp as the rest of the plan
        //geometry_msgs::PoseStamped goal_copy = goal;
        
        NS_DataType::PoseStamped goal_copy = goal;
        
        goal_copy.header.stamp = NS_NaviCommon::Time::now ();
        plan.push_back (goal_copy);
      }
      else
      {
        // 错误提示
        NS_NaviCommon::console.error (
            "Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
      }
    }
    else
    {
      // 错误提示
      NS_NaviCommon::console.error ("Failed to get a plan.");
    }
    
    // add orientations if needed
    orientation_filter_->processPath (start, plan);
    
    delete potential_array_;
    return !plan.empty (); // plan 非空即制订了 plan，返回 true
  }
  
  void
  GlobalPlanner::clearRobotCell (unsigned int mx, unsigned int my)
  {
    if (!initialized_)
    {
      // 错误提示
      NS_NaviCommon::console.error (
          "This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }
    
    //set the associated costs in the cost map to be free
    costmap->getLayeredCostmap ()->getCostmap ()->setCost (
        mx, my, NS_CostMap::FREE_SPACE);
  }
  
  bool
  GlobalPlanner::getPlanFromPotential (
      double start_x, double start_y, double goal_x, double goal_y,
      const NS_DataType::PoseStamped& goal,
      std::vector<NS_DataType::PoseStamped>& plan)
  {
    if (!initialized_)
    {
      // 错误提示
      NS_NaviCommon::console.error (
          "This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }
    
//    std::string global_frame = frame_id_;
    
    //clear the plan, just in case
    plan.clear ();
    
    std::vector<std::pair<float, float> > path;
    
    if (!path_maker_->getPath (potential_array_, start_x, start_y, goal_x,
                               goal_y, path))
    {
      // 错误提示
      NS_NaviCommon::console.error ("NO PATH!");
      return false;
    }
    
    NS_NaviCommon::Time plan_time = NS_NaviCommon::Time::now ();
    for (int i = path.size () - 1; i >= 0; i--)
    {
      std::pair<float, float> point = path[i];
      //convert the plan to world coordinates
      double world_x, world_y;
      mapToWorld (point.first, point.second, world_x, world_y);
      
      NS_DataType::PoseStamped pose;
      
      pose.header.stamp = plan_time;
//        pose.header.frame_id = global_frame;
      pose.pose.position.x = world_x;
      pose.pose.position.y = world_y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      plan.push_back (pose);
    }
    
    return !plan.empty ();
  }
  
  void
  GlobalPlanner::mapToWorld (double mx, double my, double& wx, double& wy)
  {
    wx = costmap->getLayeredCostmap ()->getCostmap ()->getOriginX ()
        + (mx + convert_offset_)
            * costmap->getLayeredCostmap ()->getCostmap ()->getResolution ();
    wy = costmap->getLayeredCostmap ()->getCostmap ()->getOriginY ()
        + (my + convert_offset_)
            * costmap->getLayeredCostmap ()->getCostmap ()->getResolution ();
  }
  
  bool
  GlobalPlanner::worldToMap (double wx, double wy, double& mx, double& my)
  {
    double origin_x =
        costmap->getLayeredCostmap ()->getCostmap ()->getOriginX (), origin_y =
        costmap->getLayeredCostmap ()->getCostmap ()->getOriginY ();
    double resolution =
        costmap->getLayeredCostmap ()->getCostmap ()->getResolution ();
    
    if (wx < origin_x || wy < origin_y)
      return false;
    
    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;
    
    if (mx < costmap->getLayeredCostmap ()->getCostmap ()->getSizeInCellsX ()
        && my
            < costmap->getLayeredCostmap ()->getCostmap ()->getSizeInCellsY ())
      return true;
    
    return false;
  }

} /* namespace NS_Planner */
