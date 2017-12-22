#ifndef SMOOTHER_H
#define SMOOTHER_H

#include "waypoint.h"

/// A simple smoother
class Smoother
{
public:
    Smoother() : isInit_(false), timeConstant_(0) {}

    ~Smoother() {}

    void reset() { isInit_ = false; }

    void setTimeConstant(double seconds) { timeConstant_ = seconds; }

    inline const CartesianCoord& filter(double t, const CartesianCoord& signal);

    inline const CartesianCoord& getLastValue() const { return smoothed_; }

private:
    bool      isInit_;
    double    timeConstant_;
    double    lastTime_;
    CartesianCoord  smoothed_;
};


//---------------------------------------------------------------------------------------------------------------------
const CartesianCoord& Smoother::filter(double t, const CartesianCoord& signal)
//---------------------------------------------------------------------------------------------------------------------
{
    if(!isInit_)
    {
        smoothed_ = signal;
        lastTime_ = t;
        isInit_ = true;
    }
    else
    {
        double period = t - lastTime_; ///@note: period can be 0
        smoothed_.x = ((signal.x * period) + (timeConstant_ * smoothed_.x))/(timeConstant_ + period);
        smoothed_.y = ((signal.y * period) + (timeConstant_ * smoothed_.y))/(timeConstant_ + period);
        lastTime_ = t;
    }
    return smoothed_;
}

#endif // SMOOTHER_H
