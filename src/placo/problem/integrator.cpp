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

Expression Integrator::expr(int step, int diff)
{
  if (diff < 0 || diff > order)
  {
    std::ostringstream oss;
    oss << "Asked differentiation order of " << diff << " for an integrator of order " << order;
    throw std::runtime_error(oss.str());
  }

  if (diff == order)
  {
    // At order, we just select the relevant variable
    return variable.expr(step, 1);
  }
  else
  {
    Expression e;

    e.A = Eigen::MatrixXd(1, variable.k_end);
    e.A.setZero();
    e.b = Eigen::VectorXd(1);

    e.A.block(0, variable.k_start, 1, step) = final_transition_matrix.block(diff, N - step, 1, step);

    e.b(0, 0) = (a_powers[step] * X0)(diff, 0);

    return e;
  }
}

}  // namespace placo