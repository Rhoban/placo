#pragma once

#include <string>
#include <vector>
#include "placo/problem/expression.h"
#include "placo/problem/variable.h"

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

  Variable& add_variable(std::string name, int size = 1);

  /**
   * @brief Adds an equality constraint (Ax + b = 0)
   * @param expression
   * @return The constraint
   */
  Constraint& add_equality_zero(Expression expression);

  /**
   * @brief Adds an equality constraint (Ax + b = t)
   * @param expression 
   * @param target 
   * @return The constraint
   */
  Constraint& add_equality(Expression expression, Eigen::VectorXd target);

  /**
   * @brief Adds an inequality constraint (Ax + b >= 0)
   * @param expression
   * @return The constraint
   */
  Constraint& add_greater_than_zero(Expression expression);

  /**
   * @brief Adds an inequality constraint (Ax + b <= 0)
   * @param expression
   * @return The constraint
   */
  Constraint& add_lower_than_zero(Expression expression);

  /**
   * @brief Adds an inequality constraint (Ax + b >= t)
   * @param expression 
   * @param target 
   * @return The constraint
   */
  Constraint& add_greater_than(Expression expression, Eigen::VectorXd target);

  /**
   * @brief Adds an inequality constraint (Ax + b <= t)
   * @param expression 
   * @param target 
   * @return The constraint
   */
  Constraint& add_lower_than(Expression expression, Eigen::VectorXd target);

  /**
   * @brief Adds a limit, "absolute" inequality constraint (abs(Ax + b) <= t)
   * @param expression 
   * @param target 
   * @return The constraint
   */
  void add_limit(Expression expression, Eigen::VectorXd target);

  void solve();

  std::vector<Variable*> variables;
  int n_variables = 0;

  std::vector<Constraint*> constraints;
};
}  // namespace placo