#include "placo/problem/constraints.h"

namespace placo::problem
{
ProblemConstraints::ProblemConstraints()
{
}

ProblemConstraints::ProblemConstraints(std::vector<ProblemConstraint*> constraints) : constraints(constraints)
{
}

void ProblemConstraints::configure(std::string type, double weight)
{
  configure(type == "hard", weight);
}

void ProblemConstraints::configure(bool hard, double weight)
{
  for (auto& constraint : constraints)
  {
    constraint->configure(hard ? ProblemConstraint::Hard : ProblemConstraint::Soft, weight);
  }
}

bool ProblemConstraints::is_active()
{
  for (auto& constraint : constraints)
  {
    if (constraint->is_active)
    {
      return true;
    }
  }

  return false;
}
}  // namespace placo::problem