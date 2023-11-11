#pragma once

#include <Eigen/Dense>
#include "placo/problem/expression.h"

namespace placo::problem
{
/**
 * @brief Represents a constraint to be enforced by a \ref Problem
 *
 * Mostly, to build a constraint, you can use \ref Expression operators (like ``e1 == e2`` or ``e1 <= e2``)
 */
class ProblemConstraint
{
public:
  /**
   * @brief Constraint priority
   */
  enum Priority
  {
    /**
     * @brief The constraint can be violated (it is an objective)
     */
    Soft = 0,
    /**
     * @brief The constraint **has** to be enforced (default)
     */
    Hard = 1
  };

  /**
   * @brief The constraint type
   */
  enum Type
  {
    /**
     * @brief Constraint is an equality: Ax + b = 0
     */
    Equality = 0,
    /**
     * @brief Constraint is en inequality: Ax + b >= 0
     */
    Inequality = 1
  };

  /**
   * @brief The constraint expression (Ax + b)
   */
  Expression expression;

  /**
   * @brief Constraint type
   */
  Type type = Equality;

  /**
   * @brief Constraint priority
   */
  Priority priority = Hard;

  /**
   * @brief Constraint weight (for soft constraints)
   */
  double weight = 1.0;

  /**
   * @brief This flag will be set by the solver if the constraint is active in the optimal solution
   */
  bool is_active = false;

  /**
   * @brief Configures the constraint
   * @param priority_ priority
   * @param weight weight
   * @pyignore
   */
  void configure(Priority priority_, double weight = 1.0);

  /**
   * @brief Configures the constraint
   * @param priority priority
   * @param weight weight
   */
  void configure(std::string type, double weight = 1.0);

  bool operator==(const ProblemConstraint& other) const;
};
}  // namespace placo::problem