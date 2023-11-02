#pragma once

#include "placo/problem/problem.h"
#include "placo/dynamics/constraint.h"

namespace placo::dynamics
{
class DynamicsSolver;
class AvoidSelfCollisionsConstraint : public Constraint
{
public:
  /**
   * @brief Margin for self collisions [m]
   */
  double self_collisions_margin = 0.005;

  /**
   * @brief Distance that triggers the constraint [m]
   */
  double self_collisions_trigger = 0.01;

  virtual void add_constraint(placo::problem::Problem& problem) override;
};
}  // namespace placo::dynamics