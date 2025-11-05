#pragma once

#include <Eigen/Dense>

namespace placo::tools
{
class Polynom
{
public:
  Polynom(Eigen::VectorXd coefficients);

  /**
   * @brief Computes the coefficient in front of term of degree ``degree`` after ``derivative`` differentiations
   * @param degree degree
   * @param derivative number of differentiations
   * @return coefficient
   */
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