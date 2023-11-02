#pragma once

#include "placo/problem/problem.h"
#include "placo/dynamics/constraint.h"
#include "placo/dynamics/contacts.h"

namespace placo::dynamics
{
class DynamicsSolver;
class ReactionRatioConstraint : public Constraint
{
public:
  ReactionRatioConstraint(Contact& contact, double reaction_ratio);

  /**
   * @brief Contact
   */
  Contact& contact;

  /**
   * @brief Reaction ratio to be enforced
   */
  double reaction_ratio;

  virtual void add_constraint(problem::Problem& problem, problem::Expression& tau) override;
};
}  // namespace placo::dynamics