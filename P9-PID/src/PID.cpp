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
}

void PID::UpdateError(double cte)
{
    static double lastCte = cte;
    static double sumCte = 0;

    _errorAcc.addData(cte*cte);
    std::cout << "MSE: " << _errorAcc.mean() << std::endl;

    // P term
    _error = -_gain[0] * cte;

    // D term
    _error = -_gain[1] * (cte - lastCte);
    lastCte = cte;

    // I term
    _error = -_gain[2] * sumCte;
    sumCte += cte;
}

void PID::TwiddleError(double cte)
{
    static double bestMse = std::numeric_limits<double>::max();
    static std::array<double,3> delta = { .1 * fabs(_gain[0]), .1 * fabs(_gain[1]), .1 * fabs(_gain[2]) }; // always > 0
    static double deltaTolSum = 0.001;
    static unsigned int iController = 0; // select which controller to modify

    // compute error as usual and accumulate stats for a number of samples
    UpdateError(cte);
    if( _errorAcc.numData() < _numErrorSamples)
    {
        return;
    }
    double mse = _errorAcc.mean();
    _errorAcc.reset();

    // exit now if gain changes are going to be too small to be significant
    const double sumDelta = std::accumulate(delta.begin(), delta.end(), 0);
    if( sumDelta < deltaTolSum)
    {
        return;
    }

    // todo: select controller index

    // update gain
    _gain[iController] += delta[iController];
    bEvalError = true;
    return;

    // todo: evaluate error n times

    if( mse < bestMse )
    {
        bestMse = mse;
        delta[iController] *= 1.1;
    }
    else
    {
        _gain[iController] -= 2 * delta[iController];

        // todo: evaluate error n times

        if( mse < bestMse )
        {
            bestMse = mse;
            delta[iController] *= 1.05;
        }
        else
        {
            _gain[iController] += delta[iController];
            delta[iController] *= 0.9;
        }
    }
}


