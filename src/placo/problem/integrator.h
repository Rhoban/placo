#pragma once

#include <memory>
#include <map>
#include "placo/problem/variable.h"
#include "placo/problem/expression.h"

namespace placo
{
class Integrator
{
public:
  struct Trajectory
  {
    double value(double t, int diff);

    // A copy of the variable value
    Eigen::VectorXd variable_value;

    // (continous) system matrix so that dX = M X
    Eigen::MatrixXd M;

    // Caching the keyframes for integration
    std::map<int, Eigen::VectorXd> keyframes;

    int order;
    double dt;
    double t_start = 0.;
  };

  Integrator();

  /**
   * @brief Creates an integrator able to build expressions and values over a decision variable.
   *        With this constructor, a continuous system matrix will be used (see below)
   * @param variable variable to integrate
   * @param X0 x0 (initial state)
   * @param order order
   * @param dt delta time
   */
  Integrator(Variable& variable, Eigen::VectorXd X0, int order, double dt);

  /**
   * @brief Creates an integrator able to build expressions and values over a decision variable. A custom continuous
   *        system matrix is passed, so that:
   *
   *        dX = MX
   *
   *        Where X is the state to be integrated.
   * @param variable variable to integrate
   * @param X0 x0 (initial state)
   * @param system_matrix custom continuous sytem matrix dX = MX
   * @param dt delta time
   */
  Integrator(Variable& variable, Eigen::VectorXd X0, Eigen::MatrixXd system_matrix, double dt);

  virtual ~Integrator();

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
  static Eigen::MatrixXd upper_shift_matrix(int order);

  /**
   * @brief Computes the A and B matrices in the discrete recurrent equation:
   *
   *         X_{k+1} = A X_k + B u_k
   *
   * @param order the system order
   * @param dt the delta time for integration
   * @return a pair of matrix A and vector B
   */
  static std::pair<Eigen::MatrixXd, Eigen::VectorXd> AB_matrices(Eigen::MatrixXd& M, int order, double dt);

  /**
   * @brief Builds an expression for the given step and differentiation
   * @param step the step
   * @param diff differentiation (if -1, the expression will be a vector of size order with all orders)
   * @return an expression
   */
  Expression expr(int step, int diff = -1);

  /**
   * @brief Builds an expression for the given time and differentiation
   * @param t the time
   * @param diff differentiation (if -1, the expression will be a vector of size order with all orders)
   * @return an expression
   */
  Expression expr_t(double t, int diff = -1);

  /**
   * @brief Computes
   * @param t
   * @param diff
   * @return
   */
  double value(double t, int diff);

  // Decision variable
  Variable* variable;

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

  Trajectory get_trajectory();

  static void check_diff(int order, int diff, bool allow_all = false);

  // Time offset for output trajectory
  double t_start = 0.;

protected:
  // Keeping track of the variable version
  int version = 0;

  // Internal trajectory
  Trajectory trajectory;

  // Update the internal trajectory
  void update_trajectory();
};
}  // namespace placo