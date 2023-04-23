#pragma once

#include "placo/problem/problem.h"
#include "placo/problem/constraints.h"

namespace placo
{
class PolygonConstraint
{
public:
  /**
   * @brief Given a polygon, produces inequalities so that the given point lies inside the polygon.
   * WARNING: Polygon must be clockwise (meaning that the exterior of the shape is on the  trigonometric normal of
   * the vertices)
   */
  static ProblemConstraints add_polygon_constraint(Problem& problem, Expression& expression,
                                                   std::vector<Eigen::Vector2d> polygon, double margin = 0.);
};
}  // namespace placo