#ifndef CONTOLLER_H
#define CONTROLLER_H

#include <limits>
#include <iostream>
#include <math.h>

#include "RollingMean.h"

enum class ControllerType
{
    proportional,
    integral,
    derivative
};

/// Base controller class
template <ControllerType T>
class Controller
{
public:

    Controller() : _error(0), _gain(0), _numErrorSamples(10) {}

    virtual ~Controller() {}

    /// initialise gains and number of samples to evaluate before
    /// twiddling gains
    void Init(double gain)
    {
        _gain = gain;
    }

    void SetNumTwiddleEvalSamples(unsigned int nEvalSamples)
    {
        _numErrorSamples = nEvalSamples;
    }

    /// do the control, no twiddle
    virtual void UpdateError(double cte)
    {
        _errorAcc.addData(cte*cte);
        std::cout << "MSE: " << _errorAcc.mean() << std::endl;
    }

    /// get the control effort
    double TotalError() const { return _error; }

    /// do the control with twiddle
    inline void TwiddleError(double cte);

protected:
    virtual std::string typeString() const = 0;

    double      _error;
    double      _gain;
    RollingMean _errorAcc; // mean squared error accumulator
    int         _numErrorSamples;
};

/// Proportional Controller class
class PropController : public Controller<ControllerType::proportional>
{
public:
    PropController() : Controller<ControllerType::proportional>() {}

    virtual ~PropController() = default;

    void UpdateError(double cte) final
    {
        Controller::UpdateError(cte);
        _error = -_gain * cte;
    }

    std::string typeString() const final {return "PropController"; }
};

/// Derivative controller class
class DerivController : public Controller<ControllerType::derivative>
{
public:
    DerivController() : Controller<ControllerType::derivative>() {}

    virtual ~DerivController() = default;

    void UpdateError(double cte) final
    {
        static double lastCte = cte;
        Controller::UpdateError(cte);
        _error = -_gain * (cte - lastCte);
        lastCte = cte;
    }

    std::string typeString() const final {return "DerivController"; }
};

/// Integral controller class
class IntegController : public Controller<ControllerType::integral>
{
public:
    IntegController() : Controller<ControllerType::integral>() {}

    virtual ~IntegController() = default;

    void UpdateError(double cte) final
    {
        static double sumCte = 0;
        Controller::UpdateError(cte);
        _error = -_gain * sumCte;
        sumCte += cte;
    }

    std::string typeString() const final {return "IntegController"; }
};

template <ControllerType T>
void Controller<T>::TwiddleError(double cte)
{
    static double bestMse = cte*cte;
    static double delta = .001;
    static double deltaTol = 1e-5;
    static bool bCheckReduceGain = false;

    // compute error as usual and accumulate stat
    UpdateError(cte);
    if( _errorAcc.numData() < _numErrorSamples)
    {
        return;
    }
    double mse = _errorAcc.mean();
    _errorAcc.reset();

    // dont do anything if we have already reached limits of controller capability
    if(fabs(delta) < deltaTol)
    {
        return;
    }

    // accumulated enough stats for last control gain. Now change control in some way
    if(bCheckReduceGain)
    {
        bCheckReduceGain = false;

        if(mse < bestMse) // there was some improvement
        {
            // error reduced. increase gain
            bestMse = mse;
            delta *= 1.1;
            _gain += delta;
        }
        else
        {
            _gain += delta;
            delta *= 0.9;
        }
    }

    else
    {
        if(mse < bestMse) // there was some improvement
        {
            bestMse = mse;
            delta *= 1.1;
            _gain += delta;
        }
        else // there was no improvement
        {
            _gain -= 2*delta; // revert previous gain change and push in opposite direction
            bCheckReduceGain = true; // we will check the effect next time
        }
    }

    std::cout << "\t" << typeString() << "\tGain: " << _gain << std::endl;
    return;
}

#endif // CONTROLLER_H

