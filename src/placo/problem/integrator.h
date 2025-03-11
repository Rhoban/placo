#pragma once

#include <memory>
#include <map>
#include "placo/problem/variable.h"
#include "placo/problem/expression.h"

namespace placo::problem
{
/**
 * @brief Integrator can be used to efficiently build expressions and values over a decision variable that
 * is integrated over time with a given linear system.
 */
class Integrator
{
public:
  /**
   * @brief The trajectory can be detached after a solve to retrieve the continuous trajectory produced by
   * the solver.
   */
  struct Trajectory
  {
    /**
     * @brief Gets the value of the trajectory at a given time and differentiation
     * @param t time
     * @param diff differentiation
     * @return the value
     */
    double value(double t, int diff);

    /**
     * @brief A copy of the variable value after solve
     */
    Eigen::VectorXd variable_value;

    /**
     * @brief A copy of the (continuous) system matrix
     */
    Eigen::MatrixXd M;

    /**
     * @brief Keyframes of the trajectory for each step
     */
    std::map<int, Eigen::VectorXd> keyframes;

    /**
     * @brief Trajectory duration
     * @return duration
     */
    double duration();

    /**
     * @brief order (size of the system matrix)
     */
    int order;

    /**
     * @brief step duration
     */
    double dt;

    /**
     * @brief time offset
     */
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
  Integrator(Variable& variable, Expression X0, int order, double dt);

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
   * @pyignore
   */
  Integrator(Variable& variable, Expression X0, Eigen::MatrixXd system_matrix, double dt);

  virtual ~Integrator();

  /**
   * @brief Builds a matrix M so that the system differential equation is dX = M X
   *
   * The "X" here also includes the command.
   *
   * For example, for order 2, this will look like:
   * ```text
   *      [ dx   ]   [ 0 1 0 ]
   * dX = [ ddx  ] = [ 0 0 1 ] X
   *      [ dddx ]   [ 0 0 0 ]
   * ```
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
   * @param step the step, (if -1 the last step will be used)
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

  /**
   * @brief Decision variable
   */
  Variable* variable;

  /**
   * @brief Number of steps (variable size)
   */
  int N;

  /**
   * @brief The continuous system matrix
   */
  Eigen::MatrixXd M;

  /**
   * @brief The discrete system matrix such that \f$X_{k+1} = A X_k + B u_k\f$
   */
  Eigen::MatrixXd A;

  /**
   * @brief The discrete system matrix such that \f$X_{k+1} = A X_k + B u_k\f$
   */
  Eigen::MatrixXd B;

  /**
   * @brief Initial state
   */
  Expression X0;

  /**
   * @brief Caching the discrete matrix for the last step
   */
  Eigen::MatrixXd final_transition_matrix;

  /**
   * @brief Caching the powers of A
   */
  std::map<int, Eigen::MatrixXd> a_powers;

  /**
   * @brief Integrator order (size of the system matrix)
   */
  int order;

  /**
   * @brief Integrator time step duration
   */
  double dt;

  /**
   * @brief Retrieve a trajectory after a solve
   * @return trajectory
   */
  Trajectory get_trajectory();

  /**
   * @brief Helpers to check if a requested differentiation is valid
   * @param order order
   * @param diff requested differentiation
   * @param allow_all if true, -1 is allowed
   */
  static void check_diff(int order, int diff, bool allow_all = false);

  /**
   * @brief Time offset for the trajectory
   */
  double t_start = 0.;

protected:
  /**
   * @brief The last variable version that was used to build the internal trajectory
   */
  int version = 0;

  /**
   * @brief Last built trajectory
   */
  Trajectory trajectory;

  /**
   * @brief Updates the internal trajectory
   */
  void update_trajectory();
};
}  // namespace placo::problem