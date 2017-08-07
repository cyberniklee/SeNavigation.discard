#ifndef _ODOM_ESTIMATION_
#define _ODOM_ESTIMATION_

// bayesian filtering
#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>
#include <Transform/DataTypes.h>

#include "NonLinearAnalyticConditionalGaussianOdo.h"

namespace NS_Controller
{
  
  class OdomEstimation
  {
  public:
    OdomEstimation ();

    virtual
    ~OdomEstimation ();

    bool
    update (const NS_NaviCommon::Time& filter_time);

    void
    initialize (const NS_Transform::Transform& prior,
                const NS_NaviCommon::Time& time);

    void
    getEstimate (MatrixWrapper::ColumnVector& estimate);

    void
    getEstimate (NS_Transform::Transform& estimate);

    void
    addMeasurement (const std::string& meas_type,
                    const NS_Transform::Transform& meas);

    void
    addMeasurement (const std::string& meas_type,
                    const NS_Transform::Transform& meas,
                    const MatrixWrapper::SymmetricMatrix& covar);

  private:
    void
    angleOverflowCorrect (double& a, double ref);

    void
    decomposeTransform (const NS_Transform::StampedTransform& trans, double& x,
                        double& y, double&z, double&Rx, double& Ry, double& Rz);
    void
    decomposeTransform (const NS_Transform::Transform& trans, double& x,
                        double& y, double&z, double&Rx, double& Ry, double& Rz);

    BFL::AnalyticSystemModelGaussianUncertainty* sys_model_;
    NS_Controller::NonLinearAnalyticConditionalGaussianOdo* sys_pdf_;
    BFL::LinearAnalyticConditionalGaussian* odom_meas_pdf_;
    BFL::LinearAnalyticMeasurementModelGaussianUncertainty* odom_meas_model_;
    BFL::LinearAnalyticConditionalGaussian* imu_meas_pdf_;
    BFL::LinearAnalyticMeasurementModelGaussianUncertainty* imu_meas_model_;

    BFL::Gaussian* prior_;
    BFL::ExtendedKalmanFilter* filter_;
    MatrixWrapper::SymmetricMatrix odom_covariance_, imu_covariance_;

    MatrixWrapper::ColumnVector vel_desi_, filter_estimate_old_vec_;
    NS_Transform::Transform filter_estimate_old_;
    NS_Transform::Transform odom_meas_, odom_meas_old_, imu_meas_,
        imu_meas_old_;

    NS_NaviCommon::Time filter_time_old_;
    
  };
// class

}
;
// namespace

#endif
