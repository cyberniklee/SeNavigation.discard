#ifndef _GLOBALPLANNER_H_
#define _GLOBALPLANNER_H_

#include "../../Base/GlobalPlannerBase.h"

#include "Algorithm/PotentialCalculator.h"
#include "Algorithm/Expander.h"
#include "Algorithm/Traceback.h"
#include "Algorithm/OrientationFilter.h"


namespace NS_Planner {

class Expander;
class GridPath;

class GlobalPlanner: public GlobalPlannerBase {
public:
	GlobalPlanner();
	virtual ~GlobalPlanner();

	void onInitialize();

	bool makePlan(const NS_DataType::PoseStamped& start,
			const NS_DataType::PoseStamped& goal, std::vector<NS_DataType::PoseStamped>& plan);

//	bool makePlanService(NS_ServiceType::RequestBase*  req, NS_ServiceType::ResponseBase* resp);

	bool getPlanFromPotential(double start_x, double start_y, double end_x, double end_y,
	                                  const NS_DataType::PoseStamped& goal,
	                                  std::vector<NS_DataType::PoseStamped>& plan);

//	void publishPlan(const std::vector<NS_DataType::PoseStamped>& path);

protected:
//    costmap_2d::Costmap2D* costmap_;
//    std::string frame_id_;
//    ros::Publisher plan_pub_;
    bool initialized_, allow_unknown_; //, visualize_potential_;

private:
     void mapToWorld(double mx, double my, double& wx, double& wy);
     bool worldToMap(double wx, double wy, double& mx, double& my);
     void clearRobotCell(unsigned int mx, unsigned int my);
//     void publishPotential(float* potential);

     double planner_window_x_, planner_window_y_, default_tolerance_;
//     std::string tf_prefix_;
     boost::mutex mutex_;
//     ros::ServiceServer make_plan_srv_;

     PotentialCalculator* p_calc_;
     Expander* planner_;
     Traceback* path_maker_;
     OrientationFilter* orientation_filter_;

     bool publish_potential_;
 //    ros::Publisher potential_pub_;
     int publish_scale_;

     void outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);
     unsigned char* cost_array_;
     float* potential_array_;
     unsigned int start_x_, start_y_, end_x_, end_y_;

//     bool old_navfn_behavior_; // 默认为 false
     float convert_offset_;
//     dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig> *dsrv_;
//     void reconfigureCB(global_planner::GlobalPlannerConfig &config, uint32_t level);
};

} /* namespace NS_Planner */

#endif /* NAVIGATION_PLANNER_IMPLEMENTS_GLOBALPLANNER_GLOBALPLANNER_H_ */
