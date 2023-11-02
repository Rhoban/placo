#pragma once

#include "placo/problem/problem.h"

namespace placo::kinematics
{
class KinematicsSolver;
class Constraint
{
  Constraint();

  /**
   * @brief Reference to the kinematics solver
   */
  KinematicsSolver* solver;

  virtual void add_constraint(placo::problem::Problem& problem) = 0;
};
}  // namespace placo::kinematics