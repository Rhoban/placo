#pragma once

#include "placo/problem/expression.hpp"
#include "placo/problem/variable.hpp"
#include "placo/tools/polynom.hpp"
#include <map>
#include <memory>

namespace placo::problem {
/**
 * @brief Represents a polynom for which decision variables are problem
 * coefficients
 */
class ProblemPolynom {
public:
  ProblemPolynom(Variable &variable);

  /**
   * @brief Builds a problem expression for the value of the polynom
   * @param x: abscissa
   * @param derivative differentiation order (0: p, 1: p', 2: p'' etc.)
   * @return problem expression
   */
  Expression expr(double x, int derivative = 0);

  /**
   * @brief Obtain resulting polynom (after problem is solved)
   * @return
   */
  placo::tools::Polynom get_polynom();

protected:
  // Decision variable
  Variable *variable;
};
} // namespace placo::problem