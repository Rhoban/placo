#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>
#include "placo/problem/integrator.h"

namespace placo
{
Integrator::Integrator(Variable& variable, Eigen::VectorXd X0, int order, double dt)
  : variable(variable), X0(X0), order(order), dt(dt)
{
  N = variable.size();

  // Computing the system matrix
  M = continuous_system_matrix(order);

  // Computing A and B transition matrices
  Eigen::MatrixXd Me = (M * dt).exp();
  A = Me.block(0, 0, order, order);
  B = Me.block(0, order, order, 1);

  // Computing final transition matrix and powers of A
  final_transition_matrix = Eigen::MatrixXd(order, N);
  final_transition_matrix.setZero();

  Eigen::MatrixXd Ak(order, order);
  Ak.setIdentity();
  a_powers[0] = Ak;

  for (int step = 0; step < N; step++)
  {
    final_transition_matrix.block(0, N - step - 1, order, 1) = Ak * B;
    Ak = A * Ak;
    a_powers[step + 1] = Ak;
  }
}

Eigen::MatrixXd Integrator::continuous_system_matrix(int order)
{
  Eigen::MatrixXd M(order + 1, order + 1);
  M.setZero();

  for (int k = 0; k < order; k++)
  {
    M(k, k + 1) = 1.0;
  }

  return M;
}
}  // namespace placo