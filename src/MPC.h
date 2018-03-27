#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

extern size_t N;
extern size_t x_start;
extern size_t y_start;
extern size_t delta_start;
extern size_t a_start;

class MPC {
 public:
  MPC();
  
  std::vector<long> cost_value;

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  void UpdateCost(vector<long> cost_value);
};

#endif /* MPC_H */
