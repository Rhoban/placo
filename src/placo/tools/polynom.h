#pragma once

#include <Eigen/Dense>

namespace placo::tools
{
class Polynom
{
public:
  Polynom(Eigen::VectorXd coefficients);

  static int derivative_coefficient(int degree, int derivative);

  /**
   * @brief Computes the value of polynom
   * @param x: abscissa
   * @param derivative differentiation order (0: p, 1: p', 2: p'' etc.)
   * @return value
   */
  double value(double x, int derivative = 0);

  /**
   * @brief coefficients, from highest to lowest
   */
  Eigen::VectorXd coefficients;
};
}  // namespace placo::tools