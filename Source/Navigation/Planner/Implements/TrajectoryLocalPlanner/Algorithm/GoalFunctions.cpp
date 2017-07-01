#include <Console/Console.h>
#include "GoalFunctions.h"
#include <math.h>

namespace NS_Planner {

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
        NS_NaviCommon::console.debug("Nearest waypoint to <%f, %f> is <%f, %f>\n", global_pose.getOrigin().x(), global_pose.getOrigin().y(), w.pose.position.x, w.pose.position.y);
        break;
      }
      it = plan.erase(it);
      global_it = global_plan.erase(global_it);
    }
  }

  bool transformGlobalPlan(
      NS_NaviCommon::Service* service,
      const std::vector<NS_DataType::PoseStamped>& global_plan,
      const NS_Transform::Stamped<NS_Transform::Pose>& global_pose,
      const NS_CostMap::Costmap2D& costmap,
      std::vector<NS_DataType::PoseStamped>& transformed_plan){
    transformed_plan.clear();

    if (global_plan.empty()) {
      NS_NaviCommon::console.error("Received plan with zero length");
      return false;
    }

    const NS_DataType::PoseStamped& plan_pose = global_plan[0];
    if(1) {

      NS_ServiceType::RequestTransform request_odom;
      NS_ServiceType::ResponseTransform map_transform;

      // get plan_to_global_transform from plan frame to global_frame
      // odom->map
      NS_Transform::StampedTransform plan_to_global_transform;
      NS_Transform::Transform plan_to_global_tf;
      /*
      tf.waitForTransform(global_frame, ros::Time::now(),
                          plan_pose.header.frame_id, plan_pose.header.stamp,
                          plan_pose.header.frame_id, ros::Duration(0.5));
      tf.lookupTransform(global_frame, ros::Time(),
                         plan_pose.header.frame_id, plan_pose.header.stamp, 
                         plan_pose.header.frame_id, plan_to_global_transform);
      */
      if (service->call (NS_NaviCommon::SERVICE_TYPE_MAP_ODOMETRY_TRANSFORM,
                             &request_odom, &map_transform) == false)
      {
        NS_NaviCommon::console.warning ("Get map transform failure!");
        return false;
      }
      NS_Transform::transformMsgToTF (map_transform.transform, plan_to_global_tf);
      plan_to_global_transform.setData(plan_to_global_tf);

      //let's get the pose of the robot in the frame of the plan
      NS_Transform::Stamped<NS_Transform::Pose> robot_pose;

      //TRANSFORM MAP->BASE_FOOTPRINT or BASE_LINK
      /*
      NS_Transform.transformPose(plan_pose.header.frame_id, global_pose, robot_pose);
      */
      NS_ServiceType::ResponseTransform odom_transform;
      NS_Transform::Transform global_to_base_tf;
      if (service->call (NS_NaviCommon::SERVICE_TYPE_ODOMETRY_BASE_TRANSFORM,
                             &request_odom, &odom_transform) == false)
      {
        NS_NaviCommon::console.warning ("Get odometry transform failure!");
        return false;
      }
      NS_Transform::transformMsgToTF (odom_transform.transform, global_to_base_tf);
      robot_pose.setData(plan_to_global_tf * global_to_base_tf);


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
        poseStampedTFToMsg(tf_pose, newer_pose);

        transformed_plan.push_back(newer_pose);

        double x_diff = robot_pose.getOrigin().x() - global_plan[i].pose.position.x;
        double y_diff = robot_pose.getOrigin().y() - global_plan[i].pose.position.y;
        sq_dist = x_diff * x_diff + y_diff * y_diff;

        ++i;
      }
    }

    return true;
  }

  bool getGoalPose(NS_NaviCommon::Service* service,
      const std::vector<NS_DataType::PoseStamped>& global_plan,
      NS_Transform::Stamped<NS_Transform::Pose>& goal_pose) {
    if (global_plan.empty())
    {
      NS_NaviCommon::console.error("Received plan with zero length");
      return false;
    }

    const NS_DataType::PoseStamped& plan_goal_pose = global_plan.back();
    if(1){
      //NS_Transform::StampedTransform transform;
      /*
      tf.waitForTransform(global_frame, ros::Time::now(),
                          plan_goal_pose.header.frame_id, plan_goal_pose.header.stamp,
                          plan_goal_pose.header.frame_id, ros::Duration(0.5));
      tf.lookupTransform(global_frame, ros::Time(),
                         plan_goal_pose.header.frame_id, plan_goal_pose.header.stamp,
                         plan_goal_pose.header.frame_id, transform);
      */
      NS_ServiceType::RequestTransform request_odom;
      NS_ServiceType::ResponseTransform map_transform;
      if (service->call (NS_NaviCommon::SERVICE_TYPE_MAP_ODOMETRY_TRANSFORM,
                                   &request_odom, &map_transform) == false)
      {
        NS_NaviCommon::console.warning ("Get map transform failure!");
        return false;
      }

      NS_Transform::Transform transform;

      //poseStampedMsgToTF(plan_goal_pose, goal_pose);
      NS_Transform::transformMsgToTF (map_transform.transform, transform);

      goal_pose.setData(transform * goal_pose);
    }

    return true;
  }

  bool isGoalReached(NS_NaviCommon::Service* service,
      const std::vector<NS_DataType::PoseStamped>& global_plan,
      const NS_CostMap::Costmap2D& costmap __attribute__((unused)),
      NS_Transform::Stamped<NS_Transform::Pose>& global_pose,
      const NS_DataType::Odometry& base_odom,
      double rot_stopped_vel, double trans_stopped_vel,
      double xy_goal_tolerance, double yaw_goal_tolerance){

    //we assume the global goal is the last point in the global plan
    NS_Transform::Stamped<NS_Transform::Pose> goal_pose;
    getGoalPose(service, global_plan, goal_pose);

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
