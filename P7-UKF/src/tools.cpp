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
    VectorXd rmse = VectorXd::Zero( 4 );
    const size_t estSz = estimations.size();
    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if(estSz != ground_truth.size() || estSz == 0)
    {
        std::cerr << "[CalculateRMSE] Invalid estimation or ground_truth data" << std::endl;
        return rmse;
    }


    //accumulate squared residuals
    for(unsigned int i=0; i < estSz; ++i)
    {

        VectorXd residual = estimations[i] - ground_truth[i];

        //coefficient-wise multiplication
        residual = residual.array()*residual.array();
        rmse += residual;
    }

    //calculate the mean
    rmse = rmse/estSz;

    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;
}
