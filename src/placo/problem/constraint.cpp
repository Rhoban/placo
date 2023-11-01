#include "placo/problem/constraint.h"

namespace placo::problem
{
void ProblemConstraint::configure(std::string type, double weight_)
{
  priority = (type == "hard" ? Hard : Soft);
  weight = weight_;
}

void ProblemConstraint::configure(ProblemConstraint::Priority priority_, double weight_)
{
  priority = priority_;
  weight = weight_;
}

bool ProblemConstraint::operator==(const ProblemConstraint& other) const
{
  return (expression.A == other.expression.A) && (expression.b == other.expression.b) && (priority == other.priority) &&
         (weight == other.weight) && (type == other.type);
}
}  // namespace placo::problem