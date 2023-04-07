#pragma once

#include <map>
#include "placo/problem/variable.h"
#include "placo/problem/expression.h"

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

  /**
   * @brief Computes the A and B matrices in the discrete recurrent equation:
   *
   *         X_{k+1} = A X_k + B u_k
   *
   * @param order the system order
   * @param dt the delta time for integration
   * @return a pair of matrix A and vector B
   */
  std::pair<Eigen::MatrixXd, Eigen::VectorXd> AB_matrices(int order, double dt);

  /**
   * @brief Builds an expression for the given step and differentiation
   * @param step the step
   * @param diff differentiation
   * @return an expression
   */
  Expression expr(int step, int diff);

  /**
   * @brief Computes
   * @param t
   * @param diff
   * @return
   */
  double value(double t, int diff);

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

  // Caching the keyframes for integration
  std::map<int, Eigen::VectorXd> keyframes;

  // Integrator order
  int order;

  // Time step
  double dt;

protected:
  // Keeping track of the variable version
  int version = 0;

  void update_keyframes();
};
}  // namespace placo