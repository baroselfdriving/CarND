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
    : NUM_STATES_(5),
      NUM_AUG_STATES_(7),
      NUM_SIG_PTS_(2 * NUM_AUG_STATES_ + 1)
{
  // if this is false, laser measurements will be ignored (except during init)
  useLaser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  useRadar_ = true;

  time_us_ = 0;

  // Sigma point spreading parameter
  lambda_ = 3 - NUM_AUG_STATES_;

  // initial state vector
  x_ = VectorXd::Zero(NUM_STATES_);

  // initial covariance matrix
  /// TODO: set this
  P_ = MatrixXd::Identity(NUM_STATES_, NUM_STATES_);
  P_(0,0) = 0.15;
  P_(1,1) = 0.15;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  /// TODO: set this
  std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  /// TODO: set this
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

  isInitialized_ = false;

  // predicted sigma points matrix
  xSigPred_ = MatrixXd::Zero(NUM_STATES_, NUM_SIG_PTS_);

  // Weights of sigma points
  weights_ = VectorXd(NUM_SIG_PTS_);
  weights_(0) = lambda_/(lambda_+NUM_AUG_STATES_);
  for(int i = 1; i < NUM_SIG_PTS_; ++i)
  {
      weights_(i) = 1/(2.*(lambda_ + NUM_AUG_STATES_));
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
    //std::cout << "[ProcessMeasurement]" << std::endl;

    // initialise state
    if( !isInitialized_ )
    {
        isInitialized_ = true;
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
        else
        {
            isInitialized_ = false;
        }
        time_us_ = meas_package.timestamp_;
        return;
    }

    // predict
    const double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
    time_us_ = meas_package.timestamp_;
    Prediction(dt);

    // update
    if( meas_package.sensor_type_ == MeasurementPackage::RADAR )
    {
        UpdateRadar(meas_package);
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER )
    {
        UpdateLidar(meas_package);
    }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t)
{
    //---------------------------------------------
    // Create augmented sigma points
    //---------------------------------------------

    // create augmented covariance matrix
    MatrixXd PAug = MatrixXd::Zero(NUM_AUG_STATES_, NUM_AUG_STATES_);
    MatrixXd Q = MatrixXd::Zero(2,2);
    Q(0,0) = std_a_ * std_a_;
    Q(1,1) = std_yawdd_ * std_yawdd_;
    PAug.topLeftCorner(NUM_STATES_, NUM_STATES_) = P_;
    PAug.bottomRightCorner(2,2) = Q;

    //create square root of augmented cov matrix
    MatrixXd A = PAug.llt().matrixL();

    // generate sigma points
    MatrixXd xSigAug = MatrixXd::Zero(NUM_AUG_STATES_, NUM_SIG_PTS_);
    const double rootl = sqrt(lambda_ + NUM_AUG_STATES_);

    // create augmented state vector
    VectorXd xAug = VectorXd::Zero(NUM_AUG_STATES_);
    xAug.head(NUM_STATES_) = x_;
    xSigAug.col(0) = xAug;
    for(int i = 0; i < NUM_AUG_STATES_; ++i)
    {
        const VectorXd v = rootl * A.col(i);
        xSigAug.col(i + 1) = xAug + v;
        xSigAug.col(i + 1 + NUM_AUG_STATES_) = xAug - v;
    }

    //---------------------------------------------
    // predict augmented sigma points forward
    //---------------------------------------------

    const double EEPS = 1e-5;
    for(int i = 0; i < NUM_SIG_PTS_; ++i)
    {
        const VectorXd x = xSigAug.col(i);
        VectorXd deltaX(NUM_STATES_);

        const double linAccNoiseDt = x(5) * delta_t;
        const double angAccNoiseDt = x(6) * delta_t;

        //avoid division by zero
        if(fabs(x(4)) < EEPS)
        {
            const double cdt = cos(x(3)) * delta_t;
            const double sdt = sin(x(3)) * delta_t;
            deltaX(0) = cdt * (x(2) + 0.5 * linAccNoiseDt);
            deltaX(1) = sdt * (x(2) + 0.5 * linAccNoiseDt);
        }
        else
        {
            const double cp = cos(x(3));
            const double sp = sin(x(3));
            const double dt2 = delta_t * delta_t;
            const double pn = x(3) + x(4)*delta_t;
            deltaX(0) = (x(2)/x(4) * (sin(pn) - sp)) + (0.5 * dt2 * cp * x(5));
            deltaX(1) = (x(2)/x(4) * (-cos(pn) + cp)) + (0.5 * dt2 * sp * x(5));
        }
        deltaX(2) = linAccNoiseDt;
        deltaX(3) = (x(4) * delta_t) + 0.5 * delta_t * angAccNoiseDt;
        deltaX(4) = angAccNoiseDt;

        VectorXd xSig = x.head(5) + deltaX;

        xSigPred_.col(i) = xSig;
    }

    //---------------------------------------------
    // calculate mean and covariance of state
    //---------------------------------------------

    x_.fill(0.);
    for(int i = 0; i < NUM_SIG_PTS_; ++i)
    {
        x_ += weights_(i) * xSigPred_.col(i);
    }

    P_.fill(0.);
    for(int i = 0; i < NUM_SIG_PTS_; ++i)
    {
        VectorXd xDiff = xSigPred_.col(i) - x_;
        P_ += weights_(i) * xDiff * xDiff.transpose();
    }

    //std::cout << "[PredictionX] " << x_.transpose() << std::endl;
    //std::cout << "[PredictionP] " << P_ << std::endl;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(const MeasurementPackage &meas_package)
{
    /// No need to apply UKF here. Laser measurement function is linear.
    /// Apply standard linear KF. See commented function below for
    /// how complicated UKF gets for this case

    const MatrixXd H = MatrixXd::Identity(2,5); // measurement matrix - set top-left to identity
    MatrixXd R = MatrixXd::Identity(2,2); // set the sensor measurement noise
    R(0,0) = std_laspx_ * std_laspx_;
    R(1,1) = std_laspy_ * std_laspy_;

    const VectorXd y = meas_package.raw_measurements_ - H * x_;
    const MatrixXd Ht = H.transpose();
    const MatrixXd PHt = P_ * Ht ;
    const MatrixXd S = H * PHt + R;
    const MatrixXd Si = S.inverse();
    const MatrixXd K =  PHt * Si;

    x_ = x_ + (K * y);
    P_ = P_ - (K * H * P_);
}
/*
void UKF::UpdateLidar(const MeasurementPackage& meas_package)
{
    //--------------------------------------------------
    // predict measurement
    //--------------------------------------------------

    // transform sigma points to measurement space
    const int NUM_MEAS = 2;
    MatrixXd zSig = MatrixXd(NUM_MEAS, NUM_SIG_PTS_);
    for(int i = 0; i < NUM_SIG_PTS_; ++i)
    {
        zSig(0, i) = xSigPred_(0, i);
        zSig(1, i) = xSigPred_(1, i);
    }

    // predict measurement
    VectorXd zPred = VectorXd::Zero(NUM_MEAS);
    for(int i = 0; i < NUM_SIG_PTS_; ++i)
    {
        zPred += weights_(i) * zSig.col(i);
    }

    //---------------------------------------------------
    // update state
    //---------------------------------------------------

    // compute a differene that will be used later
    MatrixXd zDelta = MatrixXd::Zero(NUM_MEAS, NUM_SIG_PTS_);
    for(int i = 0; i < NUM_SIG_PTS_; ++i)
    {
        zDelta.col(i) = zSig.col(i) - zPred;
    }

    // compute measurement covariance matrix
    MatrixXd S = MatrixXd::Zero(NUM_MEAS, NUM_MEAS);
    S(0,0) = std_laspx_ * std_laspx_; // set the sensor measurement noise
    S(1,1) = std_laspy_ * std_laspy_;

    for(int i = 0; i < NUM_SIG_PTS_; ++i)
    {
        S += weights_(i) * zDelta.col(i) * zDelta.col(i).transpose();
    }

    // compute cross correlation matrix
    MatrixXd xzCorr = MatrixXd::Zero(NUM_STATES_, NUM_MEAS);
    for(int i = 0; i < NUM_SIG_PTS_; ++i)
    {
        // state difference
        VectorXd xDelta = xSigPred_.col(i) - x_;
        xDelta(3) = atan2(sin(xDelta(3)), cos(xDelta(3))); // normalise angle to [-pi,pi]
        xzCorr += weights_(i) * xDelta * zDelta.col(i).transpose();
    }

    //calculate Kalman gain K;
    const MatrixXd Si = S.inverse();
    const MatrixXd K = xzCorr * Si;

    // compute innovation
    VectorXd innovation = meas_package.raw_measurements_ - zPred;

    //update state mean and covariance matrix
    x_ += K * innovation;
    P_ -= K * S * K.transpose();

    // compute nis
    NIS_laser_ = innovation.transpose() * Si * innovation;

    //std::cout << x_ << std::endl;
    //std::cout << NIS_radar_ << std::endl;
}*/

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(const MeasurementPackage& meas_package)
{
    //--------------------------------------------------
    // predict measurement
    //--------------------------------------------------

    // transform sigma points to measurement space
    const int NUM_MEAS = 3;
    MatrixXd zSig = MatrixXd(NUM_MEAS, NUM_SIG_PTS_);
    for(int i = 0; i < NUM_SIG_PTS_; ++i)
    {
        const VectorXd x = xSigPred_.col(i);
        zSig(0, i) = sqrt(x(0) * x(0) + x(1) * x(1));
        zSig(1, i) = atan2(x(1), x(0));
        zSig(2, i) = x(2) * (x(0) * cos(x(3)) + x(1) * sin(x(3)))/zSig(0, i);
    }

    // predict measurement
    VectorXd zPred = VectorXd::Zero(NUM_MEAS);
    for(int i = 0; i < NUM_SIG_PTS_; ++i)
    {
        zPred += weights_(i) * zSig.col(i);
    }

    //---------------------------------------------------
    // update state
    //---------------------------------------------------

    // compute a differene that will be used later
    MatrixXd zDelta = MatrixXd::Zero(NUM_MEAS, NUM_SIG_PTS_);
    for(int i = 0; i < NUM_SIG_PTS_; ++i)
    {
        VectorXd zd = zSig.col(i) - zPred;
        zd(1) = atan2(sin(zd(1)), cos(zd(1))); // normalise angle to [-pi,pi]
        zDelta.col(i) = zd;
    }

    // compute measurement covariance matrix
    MatrixXd S = MatrixXd::Zero(NUM_MEAS,NUM_MEAS);
    S(0,0) = std_radr_ * std_radr_;
    S(1,1) = std_radphi_ * std_radphi_;
    S(2,2) = std_radrd_ * std_radrd_;
    for(int i = 0; i < NUM_SIG_PTS_; ++i)
    {
        S += weights_(i) * zDelta.col(i) * zDelta.col(i).transpose();
    }

    // compute cross correlation matrix
    MatrixXd xzCorr = MatrixXd::Zero(NUM_STATES_, NUM_MEAS);
    for(int i = 0; i < NUM_SIG_PTS_; ++i)
    {
        // state difference
        VectorXd xDelta = xSigPred_.col(i) - x_;
        xDelta(3) = atan2(sin(xDelta(3)), cos(xDelta(3))); // normalise angle to [-pi,pi]
        xzCorr += weights_(i) * xDelta * zDelta.col(i).transpose();
    }

    //calculate Kalman gain K;
    const MatrixXd Si = S.inverse();
    const MatrixXd K = xzCorr * Si;

    // compute innovation
    VectorXd innovation = meas_package.raw_measurements_ - zPred;
    innovation(1) = atan2(sin(innovation(1)), cos(innovation(1))); // normalise angle to [-pi,pi]

    //update state mean and covariance matrix
    x_ += K * innovation;
    P_ -= K * S * K.transpose();

    // compute nis
    NIS_radar_ = innovation.transpose() * Si * innovation;

    //std::cout << x_ << std::endl;
    //std::cout << NIS_radar_ << std::endl;
}
