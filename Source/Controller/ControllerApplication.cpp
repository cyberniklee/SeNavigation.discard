/*
 * ControllerApplication.cpp
 *
 *  Created on: 2016年10月15日
 *      Author: seeing
 */

#include "ControllerApplication.h"
#include <Service/ServiceType/RequestOdometry.h>
#include <Service/ServiceType/ResponseOdometry.h>
#include <Service/ServiceType/RequestTransform.h>
#include <Service/ServiceType/ResponseTransform.h>
#include <DataSet/Dispitcher.h>
#include <Console/Console.h>
#include <Service/Service.h>
#include <Transform/LinearMath/Quaternion.h>
#include <Transform/LinearMath/Vector3.h>
#include <Time/Rate.h>
#include <DataSet/DataType/PoseStamped.h>
#include <Time/Utils.h>
#include <Time/Time.h>

namespace NS_Controller
{
  static double
  odom_pose_covariance_move[] = {
      1e-3, 0, 0, 0, 0, 0,
      0, 1e-3, 0, 0, 0, 0,
      0, 0, 1e6, 0, 0, 0,
      0, 0, 0, 1e6, 0, 0,
      0, 0, 0, 0, 1e6, 0,
      0, 0, 0, 0, 0, 1e3
  };

  static double
  odom_pose_covariance_stop[] = {
      1e-9, 0, 0, 0, 0, 0,
      0, 1e-3, 1e-9, 0, 0, 0,
      0, 0, 1e6, 0, 0, 0,
      0, 0, 0, 1e6, 0, 0,
      0, 0, 0, 0, 1e6, 0,
      0, 0, 0, 0, 0, 1e-9
  };

  static double
  odom_twist_covariance_move[] = {
      1e-3, 0, 0, 0, 0, 0,
      0, 1e-3, 0, 0, 0, 0,
      0, 0, 1e6, 0, 0, 0,
      0, 0, 0, 1e6, 0, 0,
      0, 0, 0, 0, 1e6, 0,
      0, 0, 0, 0, 0, 1e3
  };

  static double
  odom_twist_covariance_stop[] = {
      1e-9, 0, 0, 0, 0, 0,
      0, 1e-3, 1e-9, 0, 0, 0,
      0, 0, 1e6, 0, 0, 0,
      0, 0, 0, 1e6, 0, 0,
      0, 0, 0, 0, 1e6, 0,
      0, 0, 0, 0, 0, 1e-9
  };

  static double
  imu_orientation_covariance[] = {
      0.0025 , 0 , 0,
      0, 0.0025, 0,
      0, 0, 0.0025
  };

  static double
  imu_angular_velocity_covariance[] = {
      0.02, 0 , 0,
      0 , 0.02, 0,
      0 , 0 , 0.02
  };

  static double
  imu_linear_acceleration_covariance[] = {
      0.04 , 0 , 0,
      0 , 0.04, 0,
      0 , 0 , 0.04
  };

  ControllerApplication::ControllerApplication ()
  {
    memset (&original_pose, 0, sizeof(original_pose));
    memset (&current_odometry, 0, sizeof(current_odometry));
  }
  
  ControllerApplication::~ControllerApplication ()
  {
    if (comm)
      delete comm;
  }
  
  void
  ControllerApplication::getPoseLoop (double frequency)
  {
    NS_NaviCommon::Rate rate (frequency);

    while (running)
    {
      original_pose = getBasePose ();
      if (use_ekf_)
      {
        estimate ();
      }
      else
      {
        boost::mutex::scoped_lock locker_ (base_lock);

        NS_Transform::Quaternion ori_trans;
        NS_DataType::Quaternion ori_msg;

        ori_msg.x = 0.0f;
        ori_msg.y = 0.0f;
        ori_msg.z = sin (original_pose.theta / 2.0f);
        ori_msg.w = cos (original_pose.theta / 2.0f);

        current_odometry.pose.position.x = original_pose.x;
        current_odometry.pose.position.y = original_pose.y;
        current_odometry.pose.orientation = ori_msg;

        NS_Transform::quaternionMsgToTF(ori_msg, ori_trans);

        current_odom_transform = NS_Transform::Transform(ori_trans, NS_Transform::Vector3(original_pose.x, original_pose.y, 0));
      }
      rate.sleep ();
    }
  }

  void
  ControllerApplication::estimate ()
  {
    NS_Transform::Quaternion qo;

    NS_DataType::Quaternion q_msg;

    q_msg.x = 0.0f;
    q_msg.y = 0.0f;
    q_msg.z = sin (original_pose.theta / 2.0f);
    q_msg.w = cos (original_pose.theta / 2.0f);

    NS_Transform::quaternionMsgToTF(q_msg, qo);

    NS_Transform::Transform odom_measure;
    odom_measure  = NS_Transform::Transform(qo, NS_Transform::Vector3(original_pose.x, original_pose.y, 0));

    //add odom measure
    MatrixWrapper::SymmetricMatrix odom_covar (6);
    if (original_pose.linear_vel == 0 && original_pose.angular_vel == 0)
    {
      for (unsigned int i = 0; i < 6; i++)
        for (unsigned int j = 0; j < 6; j++)
          odom_covar(i + 1, j + 1) = odom_pose_covariance_stop[(6 * i) + j];
    }
    else
    {
      for (unsigned int i = 0; i < 6; i++)
        for (unsigned int j = 0; j < 6; j++)
          odom_covar(i + 1, j + 1) = odom_pose_covariance_move[(6 * i) + j];
    }
    estimation.addMeasurement ("odom", odom_measure, odom_covar);

    NS_Transform::Quaternion qi;
    NS_Transform::Transform imu_measure;
    qi = NS_Transform::createQuaternionFromRPY (original_pose.roll, original_pose.pitch, original_pose.yaw);
    imu_measure = NS_Transform::Transform(qi, NS_Transform::Vector3(0,0,0));

    //add imu measure
    MatrixWrapper::SymmetricMatrix imu_covar (3);
    for (unsigned int i = 0; i < 3; i++)
      for (unsigned int j = 0; j < 3; j++)
        imu_covar(i + 1, j + 1) = imu_orientation_covariance[(3 * i) + j];
    estimation.addMeasurement ("imu", imu_measure, imu_covar);

    NS_NaviCommon::Time filter_stamp = NS_NaviCommon::Time::now ();
    if (estimation.update (filter_stamp))
    {
      NS_Transform::StampedTransform ekf_trans;

      estimation.getEstimate (ekf_trans);

      current_odom_transform = ekf_trans;

      NS_Transform::poseTFToMsg(current_odom_transform, current_odometry.pose);

      current_odom_transform.getOrigin().setZ(0.0);
    }

    if (!estimation.isInitialized ())
    {
      estimation.initialize (odom_measure, NS_NaviCommon::Time::now ());
    }
  }

  void
  ControllerApplication::odomService (NS_ServiceType::RequestBase* request,
                                      NS_ServiceType::ResponseBase* response)
  {
    NS_ServiceType::RequestOdometry* req =
        (NS_ServiceType::RequestOdometry*) request;
    NS_ServiceType::ResponseOdometry* rep =
        (NS_ServiceType::ResponseOdometry*) response;
    
    boost::mutex::scoped_lock locker_ (base_lock);
    
#ifdef USE_SIMULATOR
    x = simulator.getX ();
    y = simulator.getY ();
    theta = simulator.getTheta ();
    linear_vel = simulator.getLinearVel ();
    angular_vel = simulator.getAngularVel ();
#endif

    current_odometry.twist.linear.x = original_pose.linear_vel;
    current_odometry.twist.angular.z = original_pose.angular_vel;

    rep->odom = current_odometry;
    
  }
  
  PoseState
  ControllerApplication::getBasePose ()
  {
    PoseState p;

    boost::mutex::scoped_lock locker_ (base_lock);

    p.x = comm->getFloat64Value (BASE_REG_ODOM_X);
    p.y = comm->getFloat64Value (BASE_REG_ODOM_Y);
    p.theta = comm->getFloat64Value (BASE_REG_ODOM_THETA);
    p.linear_vel = comm->getFloat64Value (BASE_REG_ODOM_LINEAR_VEL);
    p.angular_vel = comm->getFloat64Value (BASE_REG_ODOM_ANGULAR_VEL);

    /*
    p.roll = comm->getFloat64Value (BASE_REG_IMU_ROLL);
    p.pitch = comm->getFloat64Value (BASE_REG_IMU_PITCH);
    p.yaw = comm->getFloat64Value (BASE_REG_IMU_YAW);
    */

    return p;
  }

  void
  ControllerApplication::odomTransformService (
      NS_ServiceType::RequestBase* request,
      NS_ServiceType::ResponseBase* response)
  {
    NS_ServiceType::RequestTransform* req =
        (NS_ServiceType::RequestTransform*) request;
    NS_ServiceType::ResponseTransform* rep =
        (NS_ServiceType::ResponseTransform*) response;
    
    boost::mutex::scoped_lock locker_ (base_lock);
    
#ifdef USE_SIMULATOR
    current_pose.x = simulator.getX ();
    current_pose.y = simulator.getY ();
    current_pose.theta = simulator.getTheta ();
#endif

    NS_Transform::transformTFToMsg (current_odom_transform, rep->transform);
  }
  
  void
  ControllerApplication::velocityCallback (NS_DataType::DataBase* velocity)
  {
    NS_DataType::Twist* vel = (NS_DataType::Twist*) velocity;
    
    boost::mutex::scoped_lock locker_ (base_lock);
    
    double linear = vel->linear.x;
    double angular = vel->angular.z;
    
    comm->setFloat64Value (BASE_REG_LINEAR_V, linear);
    comm->setFloat64Value (BASE_REG_ANGULAR_V, angular);
    comm->setInt32Value (BASE_REG_V_SETTED, 1);
    
#ifdef USE_SIMULATOR
    simulator.setLinearVel (linear);
    simulator.setAngularVel (angular);
#endif
    
    delete velocity;
  }
  
  void
  ControllerApplication::loadParameters ()
  {
    parameter.loadConfigurationFile ("controller.xml");
    comm_dev_name_ = parameter.getParameter ("device", "/dev/stm32");
    wheel_diameter_ = parameter.getParameter ("wheel_diameter", 0.068f);
    encoder_resolution_ = parameter.getParameter ("encoder_resolution", 16);
    gear_reduction_ = parameter.getParameter ("gear_reduction", 62);
    
    wheel_track_ = parameter.getParameter ("wheel_track", 0.265f);
    accel_limit_ = parameter.getParameter ("accel_limit", 1.0f);
    
    pid_kp_right_ = parameter.getParameter ("pid_kp_r", 20.0f);
    pid_kd_right_ = parameter.getParameter ("pid_kd_r", 12.0f);
    pid_ki_right_ = parameter.getParameter ("pid_ki_r", 0.0f);
    pid_ko_right_ = parameter.getParameter ("pid_ko_r", 50.0f);
    pid_max_right_ = parameter.getParameter ("pid_max_r", 100.0f);
    pid_min_right_ = parameter.getParameter ("pid_min_r", 0.0f);
    
    pid_kp_left_ = parameter.getParameter ("pid_kp_l", 20.0f);
    pid_kd_left_ = parameter.getParameter ("pid_kd_l", 12.0f);
    pid_ki_left_ = parameter.getParameter ("pid_ki_l", 0.0f);
    pid_ko_left_ = parameter.getParameter ("pid_ko_l", 50.0f);
    pid_max_left_ = parameter.getParameter ("pid_max_l", 100.0f);
    pid_min_left_ = parameter.getParameter ("pid_min_l", 0.0f);
    
    control_duration_ = parameter.getParameter ("control_duration", 100);
    control_timeout_ = parameter.getParameter ("control_timeout", 1000);

    if (parameter.getParameter ("use_ekf", 1) == 1)
    {
      use_ekf_ = true;
    }
    else
    {
      use_ekf_ = false;
    }
  }
  
  void
  ControllerApplication::configController ()
  {
    comm->setFloat64Value (BASE_REG_WHEEL_TRACK, wheel_track_);
    comm->setFloat64Value (BASE_REG_WHEEL_DIAMETER, wheel_diameter_);
    comm->setFloat64Value (BASE_REG_ENCODER_RESOLUTION, encoder_resolution_);
    comm->setFloat64Value (BASE_REG_GEAR_REDUCTION, gear_reduction_);
    comm->setFloat64Value (BASE_REG_ACCEL_LIMIT, accel_limit_);
    
    comm->setFloat64Value (BASE_REG_PID_KP_RIGHT, pid_kp_right_);
    comm->setFloat64Value (BASE_REG_PID_KI_RIGHT, pid_ki_right_);
    comm->setFloat64Value (BASE_REG_PID_KD_RIGHT, pid_kd_right_);
    comm->setFloat64Value (BASE_REG_PID_KO_RIGHT, pid_ko_right_);
    comm->setFloat64Value (BASE_REG_PID_MAX_RIGHT, pid_max_right_);
    comm->setFloat64Value (BASE_REG_PID_MIN_RIGHT, pid_min_right_);
    
    comm->setFloat64Value (BASE_REG_PID_KP_LEFT, pid_kp_left_);
    comm->setFloat64Value (BASE_REG_PID_KI_LEFT, pid_ki_left_);
    comm->setFloat64Value (BASE_REG_PID_KD_LEFT, pid_kd_left_);
    comm->setFloat64Value (BASE_REG_PID_KO_LEFT, pid_ko_left_);
    comm->setFloat64Value (BASE_REG_PID_MAX_LEFT, pid_max_left_);
    comm->setFloat64Value (BASE_REG_PID_MIN_LEFT, pid_min_left_);
    
    comm->setInt32Value (BASE_REG_CNTL_DURATION, control_duration_);
    comm->setInt32Value (BASE_REG_VEL_TIMEOUT, control_timeout_);
    
    comm->setInt32Value (BASE_REG_CFG_DONE, 1);
    
    NS_NaviCommon::console.debug ("finish config base parameter!");
  }
  
  bool
  ControllerApplication::checkDevice ()
  {
    unsigned int test_code = 1234;
    unsigned int test_val = 0;
    comm->setInt32Value (BASE_REG_TEST, test_code);
    NS_NaviCommon::delay (100);
    comm->setInt32Value (BASE_REG_TEST, test_code);
    test_val = comm->getInt32Value (BASE_REG_TEST);
    if (test_val != test_code)
    {
      NS_NaviCommon::console.debug (
          "test stm32 connection... check code [%d], but get [%d]!", test_code,
          test_val);
      return false;
    }
    NS_NaviCommon::console.debug ("test stm32 connection...ok!");
    
    return true;
  }
  
  void
  ControllerApplication::initialize ()
  {
    NS_NaviCommon::console.message ("controller is initializing!");
    loadParameters ();
    
    comm = new SpiComm (comm_dev_name_);
    if (!comm->open ())
    {
      NS_NaviCommon::console.error ("can't open base controller device!");
      return;
    }

#ifndef USE_SIMULATOR
    if (!checkDevice ())
    {
      NS_NaviCommon::console.error ("test base controller failure!");
      return;
    }
#endif

    configController ();
    
#ifdef USE_SIMULATOR
    simulator.initialize ();
    simulator.run ();
#endif
    
    dispitcher->subscribe (
        NS_NaviCommon::DATA_TYPE_TWIST,
        boost::bind (&ControllerApplication::velocityCallback, this, _1));
    
    service->advertise (
        NS_NaviCommon::SERVICE_TYPE_RAW_ODOMETRY,
        boost::bind (&ControllerApplication::odomService, this, _1, _2));
    
    service->advertise (
        NS_NaviCommon::SERVICE_TYPE_ODOMETRY_BASE_TRANSFORM,
        boost::bind (&ControllerApplication::odomTransformService, this, _1,
                     _2));

    initialized = true;
  }
  
  void
  ControllerApplication::run ()
  {
    NS_NaviCommon::console.message ("controller is running!");
    
    running = true;

    get_pose_thread = boost::thread (
        boost::bind (&ControllerApplication::getPoseLoop, this, control_duration_));
  }
  
  void
  ControllerApplication::quit ()
  {
    NS_NaviCommon::console.message ("controller is quitting!");
    
    running = false;

    get_pose_thread.join ();
  }

} /* namespace NS_Controller */
