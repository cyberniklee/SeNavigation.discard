#include "GoalFunctions.h"
#include <Console/Console.h>

namespace base_local_planner {

  double getGoalPositionDistance(const NS_Transform::Stamped<NS_Transform::Pose>& global_pose, double goal_x, double goal_y) {
    return hypot(goal_x - global_pose.getOrigin().x(), goal_y - global_pose.getOrigin().y());
  }

  double getGoalOrientationAngleDifference(const NS_Transform::Stamped<NS_Transform::Pose>& global_pose, double goal_th) {
    double yaw = NS_Transform::getYaw(global_pose.getRotation());
    return NS_Geometry::NS_Angles::shortest_angular_distance(yaw, goal_th);
  }

  void prunePlan(const NS_Transform::Stamped<NS_Transform::Pose>& global_pose, std::vector<NS_DataType::PoseStamped>& plan, std::vector<NS_DataType::PoseStamped>& global_plan){
    assert(global_plan.size() >= plan.size());
    std::vector<NS_DataType::PoseStamped>::iterator it = plan.begin();
    std::vector<NS_DataType::PoseStamped>::iterator global_it = global_plan.begin();
    while(it != plan.end()){
      const NS_DataType::PoseStamped& w = *it;
      // Fixed error bound of 2 meters for now. Can reduce to a portion of the map size or based on the resolution
      double x_diff = global_pose.getOrigin().x() - w.pose.position.x;
      double y_diff = global_pose.getOrigin().y() - w.pose.position.y;
      double distance_sq = x_diff * x_diff + y_diff * y_diff;
      if(distance_sq < 1){
        ROS_DEBUG("Nearest waypoint to <%f, %f> is <%f, %f>\n", global_pose.getOrigin().x(), global_pose.getOrigin().y(), w.pose.position.x, w.pose.position.y);
        break;
      }
      it = plan.erase(it);
      global_it = global_plan.erase(global_it);
    }
  }

  bool transformGlobalPlan(
      const NS_Transform::StampedTransform& map_transform,
      const std::vector<NS_DataType::PoseStamped>& global_plan,
      const NS_Transform::Stamped<NS_Transform::Pose>& global_pose,
      const NS_CostMap::Costmap2D& costmap,
      std::vector<NS_DataType::PoseStamped>& transformed_plan){
    transformed_plan.clear();

    if (global_plan.empty()) {
      ROS_ERROR("Received plan with zero length");
      return false;
    }

    const NS_DataType::PoseStamped& plan_pose = global_plan[0];
    try {
      // get plan_to_global_transform from plan frame to global_frame
      NS_Transform::StampedTransform plan_to_global_transform;
      tf.waitForTransform(global_frame, ros::Time::now(),
                          plan_pose.header.frame_id, plan_pose.header.stamp,
                          plan_pose.header.frame_id, ros::Duration(0.5));
      tf.lookupTransform(global_frame, ros::Time(),
                         plan_pose.header.frame_id, plan_pose.header.stamp, 
                         plan_pose.header.frame_id, plan_to_global_transform);

      //let's get the pose of the robot in the frame of the plan
      NS_Transform::Stamped<NS_Transform::Pose> robot_pose;

      //TRANSFORM MAP->BASE_FOOTPRINT or BASE_LINK
      NS_Transform.transformPose(plan_pose.header.frame_id, global_pose, robot_pose);

      //we'll discard points on the plan that are outside the local costmap
      double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                       costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);

      unsigned int i = 0;
      double sq_dist_threshold = dist_threshold * dist_threshold;
      double sq_dist = 0;

      //we need to loop to a point on the plan that is within a certain distance of the robot
      while(i < (unsigned int)global_plan.size()) {
        double x_diff = robot_pose.getOrigin().x() - global_plan[i].pose.position.x;
        double y_diff = robot_pose.getOrigin().y() - global_plan[i].pose.position.y;
        sq_dist = x_diff * x_diff + y_diff * y_diff;
        if (sq_dist <= sq_dist_threshold) {
          break;
        }
        ++i;
      }

      NS_Transform::Stamped<NS_Transform::Pose> tf_pose;
      NS_DataType::PoseStamped newer_pose;

      //now we'll transform until points are outside of our distance threshold
      while(i < (unsigned int)global_plan.size() && sq_dist <= sq_dist_threshold) {
        const NS_DataType::PoseStamped& pose = global_plan[i];
        poseStampedMsgToTF(pose, tf_pose);
        tf_pose.setData(plan_to_global_transform * tf_pose);
        tf_pose.stamp_ = plan_to_global_transform.stamp_;
        tf_pose.frame_id_ = global_frame;
        poseStampedTFToMsg(tf_pose, newer_pose);

        transformed_plan.push_back(newer_pose);

        double x_diff = robot_pose.getOrigin().x() - global_plan[i].pose.position.x;
        double y_diff = robot_pose.getOrigin().y() - global_plan[i].pose.position.y;
        sq_dist = x_diff * x_diff + y_diff * y_diff;

        ++i;
      }
    }
    catch(tf::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      if (!global_plan.empty())
        ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

      return false;
    }

    return true;
  }

  bool getGoalPose(const NS_Transform::StampedTransform& map_transform,
      const std::vector<NS_DataType::PoseStamped>& global_plan,
      NS_Transform::Stamped<NS_Transform::Pose>& goal_pose) {
    if (global_plan.empty())
    {
      ROS_ERROR("Received plan with zero length");
      return false;
    }

    const NS_DataType::PoseStamped& plan_goal_pose = global_plan.back();
    try{
      NS_Transform::StampedTransform transform;
      tf.waitForTransform(global_frame, ros::Time::now(),
                          plan_goal_pose.header.frame_id, plan_goal_pose.header.stamp,
                          plan_goal_pose.header.frame_id, ros::Duration(0.5));
      tf.lookupTransform(global_frame, ros::Time(),
                         plan_goal_pose.header.frame_id, plan_goal_pose.header.stamp,
                         plan_goal_pose.header.frame_id, transform);

      poseStampedMsgToTF(plan_goal_pose, goal_pose);
      goal_pose.setData(transform * goal_pose);
      goal_pose.stamp_ = transform.stamp_;
      goal_pose.frame_id_ = global_frame;

    }
    catch(tf::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      if (global_plan.size() > 0)
        ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

      return false;
    }
    return true;
  }

  bool isGoalReached(const NS_Transform::StampedTransform& map_transform,
      const std::vector<NS_DataType::PoseStamped>& global_plan,
      const NS_CostMap::Costmap2D& costmap __attribute__((unused)),
      NS_Transform::Stamped<NS_Transform::Pose>& global_pose,
      const NS_DataType::Odometry& base_odom,
      double rot_stopped_vel, double trans_stopped_vel,
      double xy_goal_tolerance, double yaw_goal_tolerance){

    //we assume the global goal is the last point in the global plan
    NS_Transform::Stamped<NS_Transform::Pose> goal_pose;
    getGoalPose(map_transform, global_plan, goal_pose);

    double goal_x = goal_pose.getOrigin().getX();
    double goal_y = goal_pose.getOrigin().getY();
    double goal_th = NS_Transform::getYaw(goal_pose.getRotation());

    //check to see if we've reached the goal position
    if(getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance) {
      //check to see if the goal orientation has been reached
      if(fabs(getGoalOrientationAngleDifference(global_pose, goal_th)) <= yaw_goal_tolerance) {
        //make sure that we're actually stopped before returning success
        if(stopped(base_odom, rot_stopped_vel, trans_stopped_vel))
          return true;
      }
    }

    return false;
  }

  bool stopped(const NS_DataType::Odometry& base_odom,
      const double& rot_stopped_velocity, const double& trans_stopped_velocity){
    return fabs(base_odom.twist.twist.angular.z) <= rot_stopped_velocity 
      && fabs(base_odom.twist.twist.linear.x) <= trans_stopped_velocity
      && fabs(base_odom.twist.twist.linear.y) <= trans_stopped_velocity;
  }
};
