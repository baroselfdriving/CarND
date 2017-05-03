#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
    VectorXd rmse(4);
    rmse << 0,0,0,0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if(estimations.size() != ground_truth.size() || estimations.size() == 0)
    {
        std::cerr << "[CalculateRMSE] Invalid estimation or ground_truth data" << std::endl;
        return rmse;
    }

    //accumulate squared residuals
    for(unsigned int i=0; i < estimations.size(); ++i)
    {

        VectorXd residual = estimations[i] - ground_truth[i];

        //coefficient-wise multiplication
        residual = residual.array()*residual.array();
        rmse += residual;
    }

    //calculate the mean
    rmse = rmse/estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state)
{
    MatrixXd Hj = MatrixXd::Zero(3,4);

    //recover state parameters
    const double px = x_state(0);
    const double py = x_state(1);
    const double vx = x_state(2);
    const double vy = x_state(3);

    const double d = sqrt(px*px + py*py);

    //check division by zero
    const double eeps = 1e-6;
    if(d < eeps)
    {
        std::cerr << "[CalculateJacobian] Division by zero" << std::endl;
        return Hj;
    }

    const double d2 = d*d;
    const double d32 = d*d2;
    const double pj = (vx*py - vy*px)/d32;

    Hj(0,0) = px/d;
    Hj(0,1) = py/d;
    Hj(1,0) = -py/d2;
    Hj(1,1) = px/d2;
    Hj(2,0) = py*pj;
    Hj(2,1) = -px*pj;
    Hj(2,2) = Hj(0,0);
    Hj(2,3) = Hj(0,1);

    return Hj;
}

