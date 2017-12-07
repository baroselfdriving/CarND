#ifndef PID_H
#define PID_H

#include "RollingMean.h"
#include <array>

class PID {
public:

    /*
     * PID terms
     */
    std::array<double,3> _gain;
    double _error;

    /*
     * Twiddle components
     */
    RollingMean          _errorAcc; // mean squared error accumulator
    unsigned int         _numErrorSamples;
    std::array<double,3> _deltaGain;


  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID() = default;

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
   * Set twiddle parameters
   */
  void SetNumTwiddleEvalSamples(unsigned int nEvalSamples) { _numErrorSamples = nEvalSamples; }

  /*
  * Autotune PID controller using twiddle algorithm.
  */
  void TwiddleError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError() { return _error; }
};

#endif /* PID_H */
