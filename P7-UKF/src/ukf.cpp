#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF()
{
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // initial state vector
  x_ = VectorXd::Zero(n_x_);

  // initial covariance matrix
  P_ = MatrixXd::Identity(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  //TODO: set this
  std_a_ = .30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  // TODO: set this
  std_yawdd_ = .30;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  is_initialized_ = false;

  ///TODO: intialise the following properly.
  ///
  const int n_sigma = 2 * n_aug_ + 1;

  // predicted sigma points matrix
  Xsig_pred_ = MatrixXd::Zero(n_x_, n_sigma);

  // Weights of sigma points
  weights_ = VectorXd(n_sigma);
  weights_(0) = lambda_/(lambda_+n_aug_);
  for(int i = 1; i < n_sigma; ++i)
  {
      weights_(i) = 1/(2*(lambda_ + n_aug_));
  }

  // the current NIS for radar
  NIS_radar_ = 100;

  // the current NIS for laser
  NIS_laser_ = 100;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(const MeasurementPackage& meas_package)
{
    std::cout << "[ProcessMeasurement]" << std::endl;

    // initialise state
    if( !is_initialized_ )
    {
        if( meas_package.sensor_type_ == MeasurementPackage::RADAR )
        {
            x_ << meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]),
                  meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]),
                  0, 0, 0;

        }
        else if (meas_package.sensor_type_ == MeasurementPackage::LASER )
        {
            x_ << meas_package.raw_measurements_[0],
                  meas_package.raw_measurements_[1],
                  0, 0, 0;

        }
        time_us_ = meas_package.timestamp_;
        is_initialized_ = true;
        return;
    }

    // predict
    const double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
    time_us_ = meas_package.timestamp_;
    Prediction(dt);
/*
    // update
    if( meas_package.sensor_type_ == MeasurementPackage::RADAR )
    {
        UpdateRadar(meas_package);
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER )
    {
        UpdateLidar(meas_package);
    }*/
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t)
{
    std::cout << "[Prediction]" << std::endl;

    // create augmented state vector
    VectorXd x_aug = VectorXd::Zero(n_aug_);
    x_aug.head(n_x_) = x_;

    // create augmented covariance matrix
    MatrixXd Q = MatrixXd::Zero(2,2);
    MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
    Q(0,0) = std_a_ * std_a_;
    Q(1,1) = std_yawdd_ * std_yawdd_;
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug.bottomRightCorner(2,2) = Q;

    //create square root matrix
    MatrixXd A = P_aug.llt().matrixL();

    // generate sigma points
    const int n_sigma = 2 * n_aug_ + 1;

    //create augmented sigma points
    MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sigma);
    Xsig_aug.col(0) = x_aug;
    const int pcols = P_aug.cols();
    const double r3 = sqrt(lambda_ + n_aug_);
    for(int i = 0; i < pcols; ++i)
    {
        const VectorXd v = r3*A.col(i);
        Xsig_aug.col(i+1) = x_aug + v;
        Xsig_aug.col(i+1+pcols) = x_aug - v;
    }

    // predict sigma points
    const double eeps = 1e-6;
    for(int i = 0; i < Xsig_aug.cols(); ++i)
    {
        const VectorXd x = Xsig_aug.col(i);
        VectorXd xd(n_x_);

        //avoid division by zero
        if(fabs(x(4)) < eeps)
        {
            const double cdt = cos(x(3)) * delta_t;
            const double sdt = sin(x(3)) * delta_t;
            const double adt = x(5) * delta_t;
            const double pdt = x(6) * delta_t;
            xd(0) = cdt * (x(2) + 0.5 * adt);
            xd(1) = sdt * (x(2) + 0.5 * adt);
            xd(2) = adt;
            xd(3) = 0.5 * delta_t * pdt;
            xd(4) = pdt;
        }
        else
        {
            const double cp = cos(x(3));
            const double sp = sin(x(3));
            const double dt2 = delta_t * delta_t;
            const double pn = x(3) + x(4)*delta_t;
            xd(0) = (x(2)/x(4) * (sin(pn) - sp)) + (0.5 * dt2 * cp * x(5));
            xd(1) = (x(2)/x(4) * (-cos(pn) + cp)) + (0.5 * dt2 * sp * x(5));
            xd(2) = delta_t * x(5);
            xd(3) = (x(4) * delta_t) + 0.5 * dt2 * x(6);
            xd(4) = delta_t * x(6);
        }
        //write predicted sigma points into right column
        VectorXd Xsig = x.head(5) + xd;

        //angle normalization
        //while (Xsig(3)> M_PI) Xsig(3)-=2.*M_PI;
        //while (Xsig(3)<-M_PI) Xsig(3)+=2.*M_PI;

        Xsig_pred_.col(i) = Xsig;
    }

    // predict mean and covariance
    for(int i = 0; i < n_sigma; ++i)
    {
        x_ += weights_(i) * Xsig_pred_.col(i);
    }
    for(int i = 0; i < n_sigma; ++i)
    {
        P_ += weights_(i) * (Xsig_pred_.col(i) - x_) * (Xsig_pred_.col(i) - x_).transpose();
    }

    std::cout << "[Prediction] " << x_.transpose() << std::endl;
    //std::cout << "[Prediction] " << P_ << std::endl;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(const MeasurementPackage& meas_package)
{
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(const MeasurementPackage& meas_package)
{
    std::cout << "[UpdateRadar]" << std::endl;

    //--------------------------------------------------
    // predict measurement
    //--------------------------------------------------

    // transform sigma points to measurement space
    const int n_sigma = 2 * n_aug_ + 1;
    const int n_z = 3;
    MatrixXd Zsig = MatrixXd(n_z, n_sigma);
    VectorXd z_pred = VectorXd::Zero(n_z);
    for(int i = 0; i < n_sigma; ++i)
    {
        const VectorXd x = Xsig_pred_.col(i);
        VectorXd z(n_z);
        z(0) = sqrt(x(0) * x(0) + x(1) * x(1));
        z(1) = atan2(x(1), x(0));
        z(2) = x(2) * (x(0) * cos(x(3)) + x(1) * sin(x(3)))/z(0);
        Zsig.col(i) = z;
    }

    // predict measurement
    for(int i = 0; i < n_sigma; ++i)
    {
        z_pred += weights_(i) * Zsig.col(i);
    }

    //---------------------------------------------------
    // update state
    //---------------------------------------------------

    // compute residuals
    MatrixXd z_delta = MatrixXd::Zero(n_z, n_sigma);
    for(int i = 0; i < n_sigma; ++i)
    {
        VectorXd zd = Zsig.col(i) - z_pred;

        //angle normalization
        while (zd(1)> M_PI) zd(1)-=2.*M_PI;
        while (zd(1)<-M_PI) zd(1)+=2.*M_PI;

        z_delta.col(i) = zd;
    }

    // compute measurement covariance matrix
    MatrixXd S = MatrixXd::Zero(n_z,n_z);
    S(0,0) = std_radr_ * std_radr_;
    S(1,1) = std_radphi_ * std_radphi_;
    S(2,2) = std_radrd_ * std_radrd_;
    for(int i = 0; i < n_sigma; ++i)
    {
        S += weights_(i) * z_delta.col(i) * z_delta.col(i).transpose();
    }

    // compute cross correlation matrix
    MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);
    for(int i = 0; i < n_sigma; ++i)
    {
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

        Tc += weights_(i) * x_diff * z_delta.col(i).transpose();
    }

    //calculate Kalman gain K;
    const MatrixXd Si = S.inverse();
    const MatrixXd K = Tc * Si;

    // compute innovation
    VectorXd innovation = meas_package.raw_measurements_ - z_pred;

    //angle normalization
    while (innovation(1)> M_PI) innovation(1)-=2.*M_PI;
    while (innovation(1)<-M_PI) innovation(1)+=2.*M_PI;

    //update state mean and covariance matrix
    x_ += K * innovation;
    P_ -= K * S * K.transpose();

    // compute nis
    NIS_radar_ = innovation.transpose() * Si * innovation;

    std::cout << x_ << std::endl;
    std::cout << NIS_radar_ << std::endl;
}
