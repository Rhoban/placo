#pragma once

#include "placo/problem/expression.h"
#include <string>

namespace placo
{
class Variable
{
public:
  std::string name;

  Expression expr(int start=-1, int rows=-1);

  // Variable offsets in the problem
  int k_start;
  int k_end;

  int size();

  // Value (after the problem solved the optimisation problem)
  Eigen::VectorXd value;
};
};  // namespace placo