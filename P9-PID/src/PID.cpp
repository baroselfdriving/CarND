#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() : Kp(0), Ki(0), Kd(0){}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Kd = Kd;
    this->Ki = Ki;

}

void PID::UpdateError(double cte) {
    static double lastCte = cte;
    static double sumCte = 0;
    p_error = -Kp * cte;
    i_error = -Ki * sumCte;
    d_error = -Kd * (cte - lastCte);
    lastCte = cte;
    sumCte += cte;
}

double PID::TotalError() {
    return p_error + i_error + d_error;
}

