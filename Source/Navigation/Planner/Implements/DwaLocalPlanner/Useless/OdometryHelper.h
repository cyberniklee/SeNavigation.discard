#ifndef _BASE_LOCAL_PLANNER_ODOMETRY_HELPER_H_
#define _BASE_LOCAL_PLANNER_ODOMETRY_HELPER_H_

#include <Transform/DataTypes.h>
#include <Service/Service.h>
#include <DataSet/DataType/Odometry.h>
#include <boost/thread.hpp>

namespace NS_Planner {

class OdometryHelper {
public:

  OdometryHelper(NS_NaviCommon::Service* service);
  ~OdometryHelper() {}

  void getOdom(NS_DataType::Odometry& base_odom);

  void getRobotVel(NS_Transform::Stamped<NS_Transform::Pose>& robot_vel);


private:
  NS_DataType::Odometry base_odom_;
  NS_NaviCommon::Service* service_;

};

} /* namespace base_local_planner */
#endif
