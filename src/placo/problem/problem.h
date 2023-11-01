#pragma once

#include <memory>
#include <string>
#include <vector>
#include "placo/problem/expression.h"
#include "placo/problem/variable.h"
#include "placo/problem/constraint.h"
#include "placo/problem/constraints.h"
#include "placo/problem/qp_error.h"

namespace placo
{
/**
 * @brief A problem is an object that has variables and constraints to be solved by a QP solver
 */
class Problem
{
public:
  Problem();
  virtual ~Problem();

  Variable& add_variable(int size = 1);

  /**
   * @brief Adds a limit, "absolute" inequality constraint (abs(Ax + b) <= t)
   * @param expression
   * @param target
   * @return The constraint
   */
  void add_limit(Expression expression, Eigen::VectorXd target);

  /**
   * @brief Adds a given constraint to the problem
   * @param constraint
   * @return The constraint
   */
  ProblemConstraint& add_constraint(const ProblemConstraint& constraint);

  /**
   * @brief Add constraints
   * @param constraints
   * @return ProblemConstraints, which is a contained allowing to configure all the constraint
   */
  ProblemConstraints add_constraints(const std::vector<ProblemConstraint>& constraints);

  /**
   * @brief Clear all the constraints
   */
  void clear_constraints();

  /**
   * @brief Clear all the variables
   */
  void clear_variables();

  void solve();

  std::vector<Variable*> variables;

  // Problem variables (real)
  int n_variables = 0;
  int n_inequalities = 0;
  int n_equalities = 0;

  // Number of QP variables used
  int qp_variables = 0;

  // Number of determined variables
  int determined_variables = 0;

  // Result
  Eigen::VectorXd x;
  Eigen::VectorXd slacks;

  // Should sparsity be used ?
  bool use_sparsity = true;

  // Should the equalities be rewritten using the QR decomposition ?
  bool rewrite_equalities = true;

  std::vector<ProblemConstraint*> constraints;

  void dump_status();

protected:
  // QR decomposition for equality constraints
  Eigen::ColPivHouseholderQR<Eigen::Matrix<double, -1, -1, 1, -1, -1>> QR;

  // Determined variables values
  Eigen::MatrixXd y;

  void get_constraint_expressions(ProblemConstraint* constraint, Eigen::MatrixXd& A, Eigen::MatrixXd& b);
};
}  // namespace placo