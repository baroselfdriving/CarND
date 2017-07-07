#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    _pControl.Init(Kp);
    _dControl.Init(Kd);
    _iControl.Init(Ki);
}

void PID::UpdateError(double cte) {
    _pControl.UpdateError(cte);
    _dControl.UpdateError(cte);
    _iControl.UpdateError(cte);
}

void PID::SetNumTwiddleEvalSamples(unsigned int nEvalSamples)
{
   _pControl.SetNumTwiddleEvalSamples(nEvalSamples);
   _iControl.SetNumTwiddleEvalSamples(nEvalSamples);
   _dControl.SetNumTwiddleEvalSamples(nEvalSamples);
}

void PID::TwiddleError(double cte) {
    _pControl.TwiddleError(cte);
    _dControl.TwiddleError(cte);
    _iControl.TwiddleError(cte);
}

double PID::TotalError() {
    return _pControl.TotalError()
            + _iControl.TotalError()
            + _dControl.TotalError();
}

