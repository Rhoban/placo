#pragma once

#include "placo/problem/problem.h"
#include "placo/tools/prioritized.h"

namespace placo::kinematics
{
class KinematicsSolver;
class Constraint : public tools::Prioritized
{
public:
  /**
   * @brief Reference to the kinematics solver
   */
  KinematicsSolver* solver = nullptr;

  /**
   * @brief true if this object memory is in the solver (it will be deleted by the solver)
   */
  bool solver_memory = false;

  virtual void add_constraint(placo::problem::Problem& problem) = 0;
};
}  // namespace placo::kinematics