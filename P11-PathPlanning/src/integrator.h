#ifndef INTEGRATOR_H
#define INTEGRATOR_H

namespace sdcnd_t3p1
{

/// A simple trapezoidal integrator, good enough for this project
class Integrator
{
public:
  Integrator(double dt) : dt_(dt), x_(0), prevXd_(0) {}

  ~Integrator() = default;

  inline void reset(double x, double xd = 0) { x_ = x; prevXd_ = xd; }

  inline double integrate(double xd)
  {
    x_ += 0.5 * (xd + prevXd_) * dt_;
    prevXd_ = xd;
    return x_;
  }

private:
  double dt_;
  double x_;
  double prevXd_;
};

}
#endif // INTEGRATOR_H
