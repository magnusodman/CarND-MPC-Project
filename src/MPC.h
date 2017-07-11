#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
private:
    vector<double> traj_x;
    vector<double> traj_y;
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

    vector<double, allocator<double>> GetTrajectoryX();

    vector<double, allocator<double>> GetTrajectoryY();
};

#endif /* MPC_H */
