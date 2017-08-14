#ifndef _POSE_INFO_CONTAINER_H_
#define _POSE_INFO_CONTAINER_H_

#include <Transform/DataTypes.h>
#include <DataSet/DataType/PoseStamped.h>
#include <DataSet/DataType/PoseWithCovarianceStamped.h>

#include <Eigen/Core>

class PoseInfoContainer
{
public:
  
  void
  update (const Eigen::Vector3f& slamPose, const Eigen::Matrix3f& slamCov,
          const NS_NaviCommon::Time& stamp, const std::string& frame_id);

  const NS_DataType::PoseStamped&
  getPoseStamped ()
  {
    return stampedPose_;
  }
  ;
  const NS_DataType::PoseWithCovarianceStamped&
  getPoseWithCovarianceStamped ()
  {
    return covPose_;
  }
  ;
  const NS_Transform::Transform&
  getTfTransform ()
  {
    return poseTransform_;
  }
  ;

protected:
  NS_DataType::PoseStamped stampedPose_;
  NS_DataType::PoseWithCovarianceStamped covPose_;
  NS_Transform::Transform poseTransform_;
  
};

#endif

