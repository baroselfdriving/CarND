#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict()
{
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z)
{
    const VectorXd y = z - H_ * x_;
    UpdateOnInnovation(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z)
{
    const double d = sqrt(x_[0] * x_[0] + x_[1] + x_[1]);
    const double phi = atan2(x_[1], x_[0]);
    const double dd = (d < 0.0001)?(0):((x_[0] * x_[2] + x_[1] * x_[3])/d);
    VectorXd expected(3);
    expected << d, phi, dd;

    VectorXd y = z - expected;
    y[1] = atan2(sin(y[1]), cos(y[1])); // elegant but expensive way to ensure phi error is in [-pi,pi]
    UpdateOnInnovation(y);
}

void KalmanFilter::UpdateOnInnovation(const Eigen::VectorXd &y)
{
    const MatrixXd Ht = H_.transpose();
    const MatrixXd S = H_ * P_ * Ht + R_;
    const MatrixXd Si = S.inverse();
    const MatrixXd K =  P_ * Ht * Si;

    x_ = x_ + (K * y);
    P_ = P_ - (K * H_ * P_);
}
