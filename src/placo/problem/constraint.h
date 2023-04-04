#pragma once

#include <Eigen/Dense>
#include "placo/problem/expression.h"

namespace placo
{
class ProblemConstraint
{
public:
  // Equality: Ax + b = 0
  // Inequality: Ax + b >= 0
  Expression expression;

  // Inequality ?
  bool inequality = false;

  // Constraint type
  bool hard = true;
  double weight = 1.0;

  void configure(std::string type, double weight);
  void configure(bool hard, double weight);
};
}  // namespace placo