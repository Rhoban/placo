#include "placo/dynamics/reaction_ratio_constraint.h"
#include "placo/dynamics/dynamics_solver.h"

namespace placo::dynamics
{
ReactionRatioConstraint::ReactionRatioConstraint(Contact& contact, double reaction_ratio)
  : contact(contact), reaction_ratio(reaction_ratio)
{
}

void ReactionRatioConstraint::add_constraint(problem::Problem& problem, problem::Expression& tau)
{
  // If a contact has a reaction ratio constraint, we want:
  // (f_z_1) / (f_z_1 + f_z_2 + ...) <= lambda
  // 0 <= - (f_z_1) ( 1 - lambda) + lambda * (f_z_2 + ...)
  double lambda = reaction_ratio;
  problem::Expression e = -contact.f.slice(2, 1) * (1 - lambda);

  for (auto& other_contact : solver->contacts)
  {
    if (!other_contact->is_internal())
    {
      e = e + lambda * other_contact->f.slice(2, 1);
    }
  }

  problem.add_constraint(e >= 0).configure(
      priority == Priority::Soft ? problem::ProblemConstraint::Soft : problem::ProblemConstraint::Hard, weight);
}
};  // namespace placo::dynamics