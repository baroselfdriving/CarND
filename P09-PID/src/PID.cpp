#include <limits>
#include <numeric>
#include <math.h>
#include <iostream>

#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID()
    : _gain({0,0,0}),
      _error(0),
      _numErrorSamples(0)
{}

void PID::Init(double Kp, double Ki, double Kd)
{
    _gain[0] = Kp;
    _gain[1] = Kd;
    _gain[2] = Ki;

    _deltaGain[0] = .1 * fabs(_gain[0]);
    _deltaGain[1] = .1 * fabs(_gain[1]);
    _deltaGain[2] = .1 * fabs(_gain[2]); // always > 0
}

void PID::UpdateError(double cte)
{
    static double lastCte = cte;
    static double sumCte = 0;

    _errorAcc.addData(cte*cte);

    // P term
    _error = -_gain[0] * cte;

    // D term
    _error += -_gain[1] * (cte - lastCte);
    lastCte = cte;

    // I term
    _error += -_gain[2] * sumCte;
    sumCte += cte;
}

void PID::TwiddleError(double cte)
{
    static unsigned int iController = 0; // select which controller to modify

    static double bestMse = std::numeric_limits<double>::max();
    static const double deltaTolSum = 1e-5;

    static int passNumber = 0; // 0,1,2

    // compute error as usual and accumulate stats for a number of samples
    UpdateError(cte);
    if( _errorAcc.numData() < _numErrorSamples)
    {
        return;
    }

    // what's the mse now
    double mse = _errorAcc.mean();
    _errorAcc.reset();
    //std::cout << "MSE: " << mse << std::endl;

    const double deltaSum = _deltaGain[0] + _deltaGain[1] + _deltaGain[2];
    //std::cout << "sum(delta) " << deltaSum << std::endl;
    if( deltaSum < deltaTolSum )
    {
        return;
    } // delta is not tiny

    switch (passNumber)
    {
    case 0:
        //std::cout << "gain up g(" << iController << ")= " << _gain[iController] << std::endl;
        _gain[iController] += _deltaGain[iController];
        break;
    case 1:
        if(mse < bestMse)
        {
            bestMse = mse;
            _deltaGain[iController] *= 1.1;
            //std::cout << "delta up d(" << iController << ")= " << _deltaGain[iController] << std::endl;
        }
        else
        {
            //std::cout << "gain invert g(" << iController << ")= " << _gain[iController] << std::endl;
            _gain[iController] -= 2*_deltaGain[iController];
        }
        break;
    case 2:
        if(mse < bestMse)
        {
            bestMse = mse;
            _deltaGain[iController] *= 1.05;
            //std::cout << "delta up d(" << iController << ")= " << _deltaGain[iController] << std::endl;
        }
        else
        {
            //std::cout << "gain cancel g(" << iController << ")= " << _gain[iController] << std::endl;
            _gain[iController] += _deltaGain[iController];
            _deltaGain[iController] *= 0.95;
            //std::cout << "delta down d(" << iController << ")= " << _deltaGain[iController] << std::endl;
        }
        break;
    default:
        std::cout << "error " << std::endl;
    }

    passNumber = (passNumber+1) % 3;
    if( passNumber == 2)
    {
        iController = (iController+1) % 3;
    }

    std::cout << "PID gains: [" << _gain[0] << " " << _gain[2] << " " << _gain[1]
              << "] MSE: " << mse << std::endl;
}


