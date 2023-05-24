#include "placo/problem/constraints.h"

namespace placo
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
    configure(hard, weight);
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
}  // namespace placo