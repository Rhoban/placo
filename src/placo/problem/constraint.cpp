#include "placo/problem/constraint.h"

namespace placo
{
void ProblemConstraint::configure(std::string type, double weight_)
{
  hard = type == "hard";
  weight = weight_;
}

void ProblemConstraint::configure(bool hard_, double weight_)
{
  hard = hard_;
  weight = weight_;
}

bool ProblemConstraint::operator==(const ProblemConstraint& other) const
{
  return (expression.A == other.expression.A) && (expression.b == other.expression.b) && (hard == other.hard) &&
         (weight == other.weight) && (inequality == other.inequality);
}
}  // namespace placo