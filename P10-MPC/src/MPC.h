#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC
{
public:
    struct output
    {
        Eigen::VectorXd x;
        Eigen::VectorXd y;
        Eigen::VectorXd psi;
        Eigen::VectorXd v;
        Eigen::VectorXd cte;
        Eigen::VectorXd epsi;
        Eigen::VectorXd delta;
        Eigen::VectorXd a;
    };

public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  output Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
