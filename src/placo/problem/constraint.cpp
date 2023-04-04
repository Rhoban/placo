#include "placo/problem/constraint.h"

namespace placo
{
void ProblemConstraint::configure(std::string type, double weight_)
{
  hard = type == "hard";
  weight = weight_;
}

}  // namespace placo