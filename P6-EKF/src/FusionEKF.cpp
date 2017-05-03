#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF()
{
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;


  ekf_.x_ = VectorXd(4); // state is [px, py, vx, vy]. intialise at first measurement
  ekf_.P_ = MatrixXd::Identity(4, 4);
  ekf_.F_ = MatrixXd::Identity(4,4); // will set non-zero elements at measurement
  ekf_.Q_ = MatrixXd::Zero(4,4); // will set non-zero elements at measurement

  // state covariance. on first measurement we know position with certainty, velocity is uncertain
  ekf_.P_(2,2) = 1000;
  ekf_.P_(3,3) = 1000;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_)
  {
    // first measurement
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        // convert polar to cartesian position. we don't know enough to determine velocity terms
        ekf_.x_ << measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]),
                measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]),
                0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
        ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  const double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  //1. Modify the F matrix so that the time is integrated
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  //2. Set the process covariance matrix Q
  const double dt2 = pow(dt,2)/2.;
  const double dt3 = pow(dt,3)/2.;

  const double noise_ax = 9; // given to us
  const double noise_ay = 9; // given to us

  ekf_.Q_(0,0) = dt2*dt2*noise_ax;
  ekf_.Q_(0,2) = dt3*noise_ax;
  ekf_.Q_(1,1) = dt2*dt2*noise_ay;
  ekf_.Q_(1,3) = dt3*noise_ay;
  ekf_.Q_(2,0) = ekf_.Q_(0,2);
  ekf_.Q_(2,2) = 2*dt2*noise_ax;
  ekf_.Q_(3,1) = ekf_.Q_(1,3);
  ekf_.Q_(3,3) = 2*dt2*noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
      ekf_.H_ = Tools::CalculateJacobian(ekf_.x_);
      ekf_.R_ = R_radar_;
      ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else
  {
      ekf_.H_ = H_laser_;
      ekf_.R_ = R_laser_;
      ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
