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
class Problem
{
public:
  Problem();
  virtual ~Problem();

  struct Constraint
  {
    // Equality: Ax + b = 0
    // Inequality: Ax + b >= 0
    Expression expression;

    // Inequality ?
    bool inequality = false;

    // Constraint type
    bool hard = true;
    double weight = 1.0;

    void configure(bool hard, double weight);
  };

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
  int n_variables = 0;

  Eigen::VectorXd slacks;

  bool use_sparsity = true;

  std::vector<ProblemConstraint*> constraints;
};
}  // namespace placo