#include "Estimation.h"

using namespace MatrixWrapper;
using namespace BFL;
using namespace std;
using namespace NS_NaviCommon;
using namespace NS_Transform;

namespace NS_Controller
{
  // constructor
  OdomEstimation::OdomEstimation ()
      : prior_ (NULL), filter_ (NULL), initialized (false)
  {
    // create SYSTEM MODEL
    ColumnVector sysNoise_Mu (6);
    sysNoise_Mu = 0;
    SymmetricMatrix sysNoise_Cov (6);
    sysNoise_Cov = 0;
    for (unsigned int i = 1; i <= 6; i++)
      sysNoise_Cov (i, i) = pow (1000.0, 2);
    Gaussian system_Uncertainty (sysNoise_Mu, sysNoise_Cov);
    sys_pdf_ = new NonLinearAnalyticConditionalGaussianOdo (system_Uncertainty);
    sys_model_ = new AnalyticSystemModelGaussianUncertainty (sys_pdf_);
    
    // create MEASUREMENT MODEL ODOM
    ColumnVector measNoiseOdom_Mu (6);
    measNoiseOdom_Mu = 0;
    SymmetricMatrix measNoiseOdom_Cov (6);
    measNoiseOdom_Cov = 0;
    for (unsigned int i = 1; i <= 6; i++)
      measNoiseOdom_Cov (i, i) = 1;
    Gaussian measurement_Uncertainty_Odom (measNoiseOdom_Mu, measNoiseOdom_Cov);
    Matrix Hodom (6, 6);
    Hodom = 0;
    Hodom (1, 1) = 1;
    Hodom (2, 2) = 1;
    Hodom (6, 6) = 1;
    odom_meas_pdf_ = new LinearAnalyticConditionalGaussian (
        Hodom, measurement_Uncertainty_Odom);
    odom_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty (
        odom_meas_pdf_);
    
    // create MEASUREMENT MODEL IMU
    ColumnVector measNoiseImu_Mu (3);
    measNoiseImu_Mu = 0;
    SymmetricMatrix measNoiseImu_Cov (3);
    measNoiseImu_Cov = 0;
    for (unsigned int i = 1; i <= 3; i++)
      measNoiseImu_Cov (i, i) = 1;
    Gaussian measurement_Uncertainty_Imu (measNoiseImu_Mu, measNoiseImu_Cov);
    Matrix Himu (3, 6);
    Himu = 0;
    Himu (1, 4) = 1;
    Himu (2, 5) = 1;
    Himu (3, 6) = 1;
    imu_meas_pdf_ = new LinearAnalyticConditionalGaussian (
        Himu, measurement_Uncertainty_Imu);
    imu_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty (
        imu_meas_pdf_);
  }
  ;
  
  // destructor
  OdomEstimation::~OdomEstimation ()
  {
    if (filter_)
      delete filter_;
    if (prior_)
      delete prior_;
    delete odom_meas_model_;
    delete odom_meas_pdf_;
    delete imu_meas_model_;
    delete imu_meas_pdf_;
    delete sys_pdf_;
    delete sys_model_;
  }
  ;
  
  // initialize prior density of filter 
  void
  OdomEstimation::initialize (const Transform& prior, const Time& time)
  {
    // set prior of filter
    ColumnVector prior_Mu (6);
    decomposeTransform (prior, prior_Mu (1), prior_Mu (2), prior_Mu (3),
                        prior_Mu (4), prior_Mu (5), prior_Mu (6));
    SymmetricMatrix prior_Cov (6);
    for (unsigned int i = 1; i <= 6; i++)
    {
      for (unsigned int j = 1; j <= 6; j++)
      {
        if (i == j)
          prior_Cov (i, j) = pow (0.001, 2);
        else prior_Cov (i, j) = 0;
      }
    }
    prior_ = new Gaussian (prior_Mu, prior_Cov);
    filter_ = new ExtendedKalmanFilter (prior_);
    
    // remember prior
    filter_estimate_old_vec_ = prior_Mu;
    filter_estimate_old_ = prior;
    filter_time_old_ = time;
    
    initialized = true;

  }
  
  // update filter
  bool
  OdomEstimation::update (const Time& filter_time)
  {
    // only update filter for time later than current filter time
    double dt = (filter_time - filter_time_old_).toSec ();
    if (dt == 0)
      return false;
    if (dt < 0)
    {
      NS_NaviCommon::console.error (
          "Will not update robot pose with time %f sec in the past.", dt);
      return false;
    }
    NS_NaviCommon::console.debug ("Update filter at time %f with dt %f",
                                  filter_time.toSec (), dt);
    
    if (!initialized)
    {
      return false;
    }

    // system update filter
    // --------------------
    // for now only add system noise
    ColumnVector vel_desi (2);
    vel_desi = 0;
    filter_->Update (sys_model_, vel_desi);
    
    // process odom measurement
    // ------------------------
    // convert absolute odom measurements to relative odom measurements in horizontal plane
    Transform odom_rel_frame = Transform (
        createQuaternionFromYaw (filter_estimate_old_vec_ (6)),
        filter_estimate_old_.getOrigin ()) * odom_meas_old_.inverse ()
        * odom_meas_;
    ColumnVector odom_rel (6);
    decomposeTransform (odom_rel_frame, odom_rel (1), odom_rel (2),
                        odom_rel (3), odom_rel (4), odom_rel (5), odom_rel (6));
    angleOverflowCorrect (odom_rel (6), filter_estimate_old_vec_ (6));
    // update filter
    odom_meas_pdf_->AdditiveNoiseSigmaSet (odom_covariance_ * pow (dt, 2));
    
    NS_NaviCommon::console.debug (
        "Update filter with odom measurement %f %f %f %f %f %f", odom_rel (1),
        odom_rel (2), odom_rel (3), odom_rel (4), odom_rel (5), odom_rel (6));
    filter_->Update (odom_meas_model_, odom_rel);
    
    odom_meas_old_ = odom_meas_;
    
    // process imu measurement
    // -----------------------
    // convert absolute imu yaw measurement to relative imu yaw measurement
    Transform imu_rel_frame = filter_estimate_old_ * imu_meas_old_.inverse ()
        * imu_meas_;
    ColumnVector imu_rel (3);
    double tmp;
    decomposeTransform (imu_rel_frame, tmp, tmp, tmp, tmp, tmp, imu_rel (3));
    decomposeTransform (imu_meas_, tmp, tmp, tmp, imu_rel (1), imu_rel (2),
                        tmp);
    angleOverflowCorrect (imu_rel (3), filter_estimate_old_vec_ (6));
    // update filter
    imu_meas_pdf_->AdditiveNoiseSigmaSet (imu_covariance_ * pow (dt, 2));
    filter_->Update (imu_meas_model_, imu_rel);
    imu_meas_old_ = imu_meas_;
    
    // remember last estimate
    filter_estimate_old_vec_ = filter_->PostGet ()->ExpectedValueGet ();
    Quaternion q;
    q.setRPY (filter_estimate_old_vec_ (4), filter_estimate_old_vec_ (5),
              filter_estimate_old_vec_ (6));
    filter_estimate_old_ = Transform (
        q,
        Vector3 (filter_estimate_old_vec_ (1), filter_estimate_old_vec_ (2),
                 filter_estimate_old_vec_ (3)));
    filter_time_old_ = filter_time;
    
    return true;
  }
  ;
  
  void
  OdomEstimation::addMeasurement (const std::string& meas_type,
                                  const Transform& meas)
  {
    if (meas_type == "odom")
      odom_meas_ = meas;
    else if (meas_type == "imu")
      imu_meas_ = meas;
    else return;
  }
  
  void
  OdomEstimation::addMeasurement (const std::string& meas_type,
                                  const Transform& meas,
                                  const MatrixWrapper::SymmetricMatrix& covar)
  {
    // check covariance
    for (unsigned int i = 0; i < covar.rows (); i++)
    {
      if (covar (i + 1, i + 1) == 0)
      {
        NS_NaviCommon::console.error (
            "Covariance specified for measurement %s is zero",
            meas_type.c_str ());
        return;
      }
    }
    // add measurements
    addMeasurement (meas_type, meas);
    if (meas_type == "odom")
      odom_covariance_ = covar;
    else if (meas_type == "imu")
      imu_covariance_ = covar;
    else return;
  }
  ;
  
  // get latest filter posterior as vector
  void
  OdomEstimation::getEstimate (MatrixWrapper::ColumnVector& estimate)
  {
    estimate = filter_estimate_old_vec_;
  }
  ;
  
  // get filter posterior at time 'time' as Stamped Transform
  void
  OdomEstimation::getEstimate (Transform& estimate)
  {
    estimate = filter_estimate_old_;
  }
  ;
  
  // correct for angle overflow
  void
  OdomEstimation::angleOverflowCorrect (double& a, double ref)
  {
    while ((a - ref) > M_PI)
      a -= 2 * M_PI;
    while ((a - ref) < -M_PI)
      a += 2 * M_PI;
  }
  ;
  
  // decompose Transform into x,y,z,Rx,Ry,Rz
  void
  OdomEstimation::decomposeTransform (const StampedTransform& trans, double& x,
                                      double& y, double&z, double&Rx,
                                      double& Ry, double& Rz)
  {
    x = trans.getOrigin ().x ();
    y = trans.getOrigin ().y ();
    z = trans.getOrigin ().z ();
    trans.getBasis ().getEulerYPR (Rz, Ry, Rx);
  }
  ;
  
  // decompose Transform into x,y,z,Rx,Ry,Rz
  void
  OdomEstimation::decomposeTransform (const Transform& trans, double& x,
                                      double& y, double&z, double&Rx,
                                      double& Ry, double& Rz)
  {
    x = trans.getOrigin ().x ();
    y = trans.getOrigin ().y ();
    z = trans.getOrigin ().z ();
    trans.getBasis ().getEulerYPR (Rz, Ry, Rx);
  }
  ;

}
;
// namespace
