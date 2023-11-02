#pragma once

#include "placo/problem/problem.h"
#include "placo/tools/prioritized.h"

namespace placo::dynamics
{
class DynamicsSolver;
class Constraint : public tools::Prioritized
{
public:
  /**
   * @brief Reference to the dynamics solver
   */
  DynamicsSolver* solver = nullptr;

  virtual void add_constraint(placo::problem::Problem& problem) = 0;
};
}  // namespace placo::dynamics