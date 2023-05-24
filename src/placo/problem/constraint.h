#pragma once

#include <Eigen/Dense>
#include "placo/problem/expression.h"

namespace placo
{
class ProblemConstraint
{
public:
  enum Priority
  {
    Soft = 0,
    Hard = 1
  };

  // Equality: Ax + b = 0
  // Inequality: Ax + b >= 0
  Expression expression;

  // Inequality ?
  bool inequality = false;

  // Constraint type
  Priority priority = Hard;
  double weight = 1.0;

  // Is this constraint active ?
  // Will be set by the solver
  bool is_active = false;

  void configure(std::string type, double weight);
  void configure(Priority priority, double weight);

  bool operator==(const ProblemConstraint& other) const;
};
}  // namespace placo