#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC
{
public:
    class Output
    {
    public:
        Output(unsigned int N)
            : x(N), y(N), psi(N), v(N), cte(N), epsi(N), delta(N), a(N)
        {}

        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> psi;
        std::vector<double> v;
        std::vector<double> cte;
        std::vector<double> epsi;
        std::vector<double> delta;
        std::vector<double> a;
    };

public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  Output Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
