#include "placo/tools/polynom.h"

namespace placo::tools
{
Polynom::Polynom(Eigen::VectorXd coefficients) : coefficients(coefficients)
{
}

int Polynom::derivative_coefficient(int degree, int derivative)
{
  if (derivative > degree)
  {
    return 0;
  }

  int coefficient = 1;

  while (derivative > 0)
  {
    coefficient *= degree;
    degree--;
    derivative--;
  }

  return coefficient;
}

double Polynom::value(double x, int differentiate)
{
  double p = 0;

  double x_pow = 1;
  for (int order = differentiate; order < coefficients.size(); order++)
  {
    p += derivative_coefficient(order, differentiate) * coefficients[coefficients.size() - order - 1] * x_pow;
    x_pow *= x;
  }

  return p;
}
}  // namespace placo::tools