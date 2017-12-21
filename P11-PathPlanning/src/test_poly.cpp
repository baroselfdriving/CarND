#include "trajectory_planner.h"
#include <iostream>
#include <deque>

//---------------------------------------------------------------------------------------------------------------------
int main()
//---------------------------------------------------------------------------------------------------------------------
{
  std::deque<TrajectoryPlanner::FrenetState> history;

  TrajectoryPlanner::PolyState initial;
  TrajectoryPlanner::PolyState final;

  double timeDelta = 10;

  //====================================================================================================================
  initial.q = 0;
  initial.qDot = 0;
  initial.qDotDot = 0;
  initial.t = 0;

  final.q = 25;
  final.qDot = 0;
  final.qDotDot = 0;
  final.t = initial.t + timeDelta;

  double t0 = 0;

  std::array<double,6> coeffs = TrajectoryPlanner::computePolynomialCoefficients(initial, final);

  double dt = 0;
  double v = 0;
  double s = 0;
  std::cout << "points1 = [";
  for(int i = 0; i < 500; ++i)
  {
    const double dt2 = dt*dt;
    const double dt3 = dt2*dt;
    const double dt4 = dt3*dt;
    const double dt5 = dt4*dt;

    TrajectoryPlanner::FrenetState fs;
    fs.sv = coeffs[0] + coeffs[1]*dt + coeffs[2]*dt2 + coeffs[3]*dt3 + coeffs[4]*dt4 + coeffs[5]*dt5;
    fs.sa = coeffs[1] + 2*coeffs[2]*dt + 3*coeffs[3]*dt2 + 4*coeffs[4]*dt3 + 5*coeffs[5]*dt4;
    fs.sj = 2*coeffs[2] + 6*coeffs[3]*dt + 12*coeffs[4]*dt2 + 20*coeffs[5]*dt3;
    fs.t = t0 + dt;

    s += 0.5 * (v + fs.sv) * TrajectoryPlanner::SIM_DELTA_TIME;
    v = fs.sv;
    fs.s = s;
    dt += TrajectoryPlanner::SIM_DELTA_TIME;

    history.push_back(fs);
  }

  for(const auto& item : history)
  {
    std::cout << item.t << ", " << item.s << ", " << item.sv << ", " << item.sa << ", " << item.sj << std::endl;
  }
  std::cout << "];";

  //====================================================================================================================
  history.erase(history.begin(), history.begin()+250);

  t0 = history.back().t;
  initial.q = history.back().sv;
  initial.qDot = history.back().sa;
  initial.qDotDot = history.back().sj;
  initial.t = history.back().t;

  final.q = 25;//history.front().s + history.back().sd * (history.front().t + timeDelta);
  final.qDot = 0;
  final.qDotDot = 0;
  final.t = initial.t + timeDelta;

  coeffs = TrajectoryPlanner::computePolynomialCoefficients(initial, final);

  dt = 0;
  v = history.back().sv;
  s = history.back().s;
  std::cout << "points2 = [";
  for(int i = 0; i < 250; ++i)
  {
    const double dt2 = dt*dt;
    const double dt3 = dt2*dt;
    const double dt4 = dt3*dt;
    const double dt5 = dt4*dt;

    TrajectoryPlanner::FrenetState fs;
    fs.sv = coeffs[0] + coeffs[1]*dt + coeffs[2]*dt2 + coeffs[3]*dt3 + coeffs[4]*dt4 + coeffs[5]*dt5;
    fs.sa = coeffs[1] + 2*coeffs[2]*dt + 3*coeffs[3]*dt2 + 4*coeffs[4]*dt3 + 5*coeffs[5]*dt4;
    fs.sj = 2*coeffs[2] + 6*coeffs[3]*dt + 12*coeffs[4]*dt2 + 20*coeffs[5]*dt3;
    fs.t = t0 + dt;

    s += 0.5 * (v + fs.sv) * TrajectoryPlanner::SIM_DELTA_TIME;
    v = fs.sv;
    fs.s = s;
    dt += TrajectoryPlanner::SIM_DELTA_TIME;

    history.push_back(fs);
  }

  for(const auto& item : history)
  {
    std::cout << item.t << ", " << item.s << ", " << item.sv << ", " << item.sa << ", " << item.sj << std::endl;
  }
  //====================================================================================================================

  std::cout << "];";


  return 0;
}
