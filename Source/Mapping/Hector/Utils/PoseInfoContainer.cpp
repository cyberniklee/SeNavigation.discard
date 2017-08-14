#include "PoseInfoContainer.h"
#include <boost/array.hpp>

void
PoseInfoContainer::update (const Eigen::Vector3f& slamPose,
                           const Eigen::Matrix3f& slamCov,
                           const NS_NaviCommon::Time& stamp,
                           const std::string& frame_id)
{
  
  NS_DataType::Pose& pose = stampedPose_.pose;
  pose.position.x = slamPose.x ();
  pose.position.y = slamPose.y ();
  
  pose.orientation.w = cos (slamPose.z () * 0.5f);
  pose.orientation.z = sin (slamPose.z () * 0.5f);
  
  //Fill covPose
  //geometry_msgs::PoseWithCovarianceStamped covPose;
  covPose_.header.stamp = stamp;
  covPose_.header.frame_id = frame_id;
  covPose_.pose = pose;
  
  covPose_.covariance[0] = static_cast<double> (slamCov (0, 0));
  covPose_.covariance[7] = static_cast<double> (slamCov (1, 1));
  covPose_.covariance[35] = static_cast<double> (slamCov (2, 2));
  
  double xyC = static_cast<double> (slamCov (0, 1));
  covPose_.covariance[1] = xyC;
  covPose_.covariance[6] = xyC;
  
  double xaC = static_cast<double> (slamCov (0, 2));
  covPose_.covariance[5] = xaC;
  covPose_.covariance[30] = xaC;
  
  double yaC = static_cast<double> (slamCov (1, 2));
  covPose_.covariance[11] = yaC;
  covPose_.covariance[31] = yaC;
  
  //Fill tf tansform
  NS_Transform::poseMsgToTF (pose, poseTransform_);
}
