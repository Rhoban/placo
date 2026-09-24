#pragma once

#include <memory>
#include <string>
#include <vector>
#include "placo/problem/expression.h"
#include "placo/problem/variable.h"
#include "placo/problem/constraint.h"
#include "placo/problem/qp_error.h"

namespace placo::problem
{
/**
 * @brief A problem is an object that has variables and constraints to be solved by a QP solver.
 */
class Problem
{
public:
  Problem();
  virtual ~Problem();

  /**
   * @brief Adds a n-dimensional variable to a problem
   * @param size dimension of the variable
   * @return variable
   */
  Variable& add_variable(int size = 1);

  /**
   * @brief Adds a limit, "absolute" inequality constraint (abs(Ax + b) <= t)
   * @param expression
   * @param target
   * @return The constraint
   */
  ProblemConstraint& add_limit(Expression expression, Eigen::VectorXd target);

  /**
   * @brief Adds a given constraint to the problem
   * @param constraint
   * @return The constraint
   */
  ProblemConstraint& add_constraint(const ProblemConstraint& constraint);

  /**
   * @brief Clear all the constraints
   */
  void clear_constraints();

  /**
   * @brief Clear all the variables
   */
  void clear_variables();

  /**
   * @brief Solves the problem, raises \ref QPError in case of failure
   */
  void solve();

  /**
   * @brief Number of problem variables that need to be solved
   */
  int n_variables = 0;

  /**
   * @brief Number of inequality constraints
   */
  int n_inequalities = 0;

  /**
   * @brief Number of equalities
   */
  int n_equalities = 0;

  /**
   * @brief Number of free variables to solve.
   *
   * If \ref rewrite_equalities is true, this should be equals to \ref n_variable.
   */
  int free_variables = 0;

  /**
   * @brief Number of slack variables in the solver.
   */
  int slack_variables = 0;

  /**
   * @brief Number of determined variables
   *
   * If \ref rewrite_equalities is true, this should be equals to 0.
   */
  int determined_variables = 0;

  /**
   * @brief Default internal regularization
   */
  double regularization = 1e-8;

  /**
   * @brief Computed result
   */
  Eigen::VectorXd x;

  /**
   * @brief Computed slack variables
   */
  Eigen::VectorXd slacks;

  /**
   * @brief If set to true, some sparsity optimizations will be performed when building the problem Hessian.
   * This optimization is generally not useful for small problems.
   */
  bool use_sparsity = true;

  /**
   * @brief If set to true, a QR factorization will be performed on the equality constraints, and the QP will be
   * called with free variables only.
   *
   * The number of free variables will be available in \ref free_variables, and the number of determined variables
   * in \ref determined_variables.
   */
  bool rewrite_equalities = true;

  void dump_status();

protected:
  /**
   * @brief Internal object to store the QR decomposition
   */
  Eigen::ColPivHouseholderQR<Eigen::Matrix<double, -1, -1, 1, -1, -1>> QR;

  /**
   * @brief Internal vector of determined values (in the Q basis)
   */
  Eigen::MatrixXd y;

  /**
   * @brief Problem variables
   */
  std::vector<Variable*> variables;

  /**
   * @brief Problem constraints
   */
  std::vector<ProblemConstraint*> constraints;

  /**
   * @brief Used internally to access a constraint expression, optionally applying the change of basis imposed by
   * the QR decomposition, see \ref rewrite_equalities.
   * @param constraint constraint
   * @param A output matrix A
   * @param b output vector b
   */
  void get_constraint_expressions(ProblemConstraint* constraint, Eigen::MatrixXd& A, Eigen::MatrixXd& b);
};
}  // namespace placo::problem