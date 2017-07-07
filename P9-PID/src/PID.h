#ifndef PID_H
#define PID_H

#include "Controller.h"

class PID {
public:
    PropController _pControl;
    DerivController _dControl;
    IntegController _iControl;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

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
  void SetNumTwiddleEvalSamples(unsigned int nEvalSamples);

  /*
  * Autotune PID controller using twiddle algorithm.
  */
  void TwiddleError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
