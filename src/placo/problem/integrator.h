#pragma once

#include <map>
#include "placo/problem/variable.h"

namespace placo
{
class Integrator
{
public:
  Integrator(Variable& variable, Eigen::VectorXd X0, int order, double dt);

  /**
   * @brief Builds a matrix M so that the system differential equation is dX = M X
   *        The "X" here also includes the command.
   *
   *        For example, for order 2, this will look like:
   *
   *             [ dx   ]   [ 0 1 0 ]
   *        dX = [ ddx  ] = [ 0 0 1 ] X
   *             [ dddx ]   [ 0 0 0 ]
   *
   * @return the matrix M
   */
  static Eigen::MatrixXd continuous_system_matrix(int order);

  // Decision variable
  Variable& variable;

  // System steps (variable size)
  int N;

  // (continous) system matrix so that dX = M X
  Eigen::MatrixXd M;

  // System matrix for the given dt, as in X_{k+1} = Ak X_k + Bk u_k
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;

  // X0
  Eigen::VectorXd X0;

  // Caching some computations
  Eigen::MatrixXd final_transition_matrix;
  std::map<int, Eigen::MatrixXd> a_powers;

  // Integrator order
  int order;

  // Time step
  double dt;
};
}  // namespace placo