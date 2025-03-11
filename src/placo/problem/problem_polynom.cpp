#include <iostream>
#include "placo/problem/problem_polynom.h"

namespace placo::problem
{
ProblemPolynom::ProblemPolynom(Variable& variable) : variable(&variable)
{
}

Expression ProblemPolynom::expr(double x, int derivative)
{
  Eigen::MatrixXd coefficients(1, variable->size());
  coefficients.setZero();

  double x_pow = 1;
  for (int order = derivative; order < variable->size(); order++)
  {
    coefficients(0, variable->size() - order - 1) =
        placo::tools::Polynom::derivative_coefficient(order, derivative) * x_pow;
    x_pow *= x;
  }

  return variable->expr().left_multiply(coefficients);
}

placo::tools::Polynom ProblemPolynom::get_polynom()
{
  return placo::tools::Polynom(variable->value);
}
}  // namespace placo::problem