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
    double phi = atan2(x_[1], x_[0]);
    phi = atan2(sin(phi), cos(phi)); // elegant but expensive way to handle discontinuities at pi

    VectorXd expected(3);
    expected << d, phi, (x_[0] * x_[2] + x_[1] * x_[3])/d;

    const VectorXd y = z - expected;
    UpdateOnInnovation(y);
}

void KalmanFilter::UpdateOnInnovation(const Eigen::VectorXd &y)
{
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K =  P_ * Ht * Si;

    x_ = x_ + (K * y);
    P_ = P_ - (K * H_ * P_);
}
