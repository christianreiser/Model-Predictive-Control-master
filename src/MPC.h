#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

#define DT 0.1 // time step duration dt in s 

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
#define LF 2.67

#define NUMBER_OF_STEPS 20 //number of timesteps
#define REF_CTE 0 // optimum cross track error
#define REF_EPSI 0 //optimium psi error
#define REF_V 32.5 //52.5 //optimum Velocity

// Set weights parameters for the cost function
#define W_CTE 25.2 //cross track error
#define W_EPSI 0.0046 //psi error
#define W_V 39 // velocity
#define W_DELTA 1200000 //smaller steering angles
#define W_A 17.1 // less acceleration
#define W_DDELTA 0.001 //less change in steering angle
#define W_DA 0.000001 // less change in acceleration

// Set lower and upper limits for variables.
#define DED25RAD 0.436332 // 25 deg in rad, used as delta bound
#define MAXTHR 1.0 // Maximal a value
#define BOUND 1.0e3 // Bound value for other variables

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  vector<double> mpc_x;
  vector<double> mpc_y;
};

#endif /* MPC_H */
