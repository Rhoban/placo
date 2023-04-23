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
}  // namespace placo